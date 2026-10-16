// ============================================================================
// 中文阅读导引（当前实现）
// 单层运算核心，阅读顺序：配置锁存/检查 → AGU与Activation Reader → Weight Loader → Mesh → Accumulator → 后处理/写回 → 状态机。
// 循环顺序为output_group → k_group → m；8个输出通道并行，每个k_group包含4个规约元素。
// issued_count统计已接收的激活地址请求，received_count统计已接收的部分和，两者不必同拍变化。
// Shadow缓存下一k_group的权重，当前组结束并且累加器流水空闲后才切换，避免污染在途计算。
// ============================================================================
//=============================================================================
// 模块：conv1d_controller
// 功能：单层 Conv1d 的总控制器，并连接 AGU、存储桥、Mesh、Accumulator
//       及后处理数据通路。
//
// 循环顺序严格为：
//   for output_group        每组并行处理 8 个输出通道
//     load bias/requant
//     for k_group           每组展开 K 维的 4 个元素
//       load 4x8 weights
//       for m               在全部输出时间点复用当前权重块
//     for m                 读取累加器、后处理并写回
//
// valid/ready 只有同时为 1 的时钟沿才算一次传输。所有计数器也只在对应
// 握手成功时更新，因此 SRAM 或输出端任意 backpressure 都不会丢数据。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - Dense主循环为output_group -> k_group -> m；同一4x8权重块复用于所有输出时间位置。
// - Activation经Reader、Skew和Mesh形成8路INT20部分和，再由Accumulator跨k_group累加为INT32。
// - Active/Shadow权重寄存器预取下一k_group，只有当前流水完全排空后才允许swap。
// - 最后依次执行逐Cout参数后处理、ReLU/ELU和8-Byte写回，所有级均支持反压。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：Dense/Pointwise Conv1d主控制器。
// 关键变量/接口：循环顺序为output_group→k_group→m；权重固定后跨m复用，控制AGU、Mesh、累加和写回。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module conv1d_controller #(
    parameter ADDR_WIDTH = 32,
    parameter M_TAG_WIDTH = 9,
    parameter MAX_M = 320,
    // 0 为原始常开时钟基线；1 为方案 V1 的整 Mesh 时钟门控。
    parameter MESH_CLOCK_GATE_ENABLE = 1
) (
    input                                       clk,
    input                                       rst_n,
    // 工艺DFT接口：功能模式由上层显式接1'b0，扫描模式接DFT控制，禁止悬空。
    // 使用普通wire端口兼容ASIC综合前端，不依赖仿真器的隐式弱下拉。
    input wire                                  mesh_scan_test_en,
    input                                       start,
    output reg                                  busy,
    output reg                                  done,
    output reg                                  error,
    input              [31:0]                   cfg_input_base,
    input              [31:0]                   cfg_output_base,
    input              [31:0]                   cfg_weight_base,
    input              [8:0]                    cfg_cin,
    input              [8:0]                    cfg_cout,
    input              [4:0]                    cfg_kernel,
    input              [3:0]                    cfg_stride,
    input              [3:0]                    cfg_dilation,
    input              [7:0]                    cfg_left_pad,
    input              [8:0]                    cfg_input_length,
    input              [8:0]                    cfg_output_length,
    input              [11:0]                   cfg_k_total,
    input              [9:0]                    cfg_k_groups,
    input              [5:0]                    cfg_n_groups,
    input                                       cfg_elu_enable,
    input                                       cfg_relu_enable,

    // ELU 负半轴查找表配置口。仅 IDLE 状态接受写入。
    input                                       elu_lut_cfg_valid,
    output                                      elu_lut_cfg_ready,
    input              [6:0]                    elu_lut_cfg_addr,
    input              [7:0]                    elu_lut_cfg_wdata,
    output                                      act_rd_req_valid,
    input                                       act_rd_req_ready,
    output             [ADDR_WIDTH-1:0]         act_rd_addr0,
    output             [ADDR_WIDTH-1:0]         act_rd_addr1,
    output             [ADDR_WIDTH-1:0]         act_rd_addr2,
    output             [ADDR_WIDTH-1:0]         act_rd_addr3,
    output             [3:0]                    act_rd_mask,
    input                                       act_rd_rsp_valid,
    output                                      act_rd_rsp_ready,
    input              [63:0]                   act_rd_data,
    output                                      wgt_rd_req_valid,
    input                                       wgt_rd_req_ready,
    output             [ADDR_WIDTH-1:0]         wgt_rd_addr,
    input                                       wgt_rd_rsp_valid,
    output                                      wgt_rd_rsp_ready,
    input              [255:0]                  wgt_rd_data,
    output                                      param_rd_req_valid,
    input                                       param_rd_req_ready,
    output             [5:0]                    param_rd_output_group,
    input                                       param_rd_rsp_valid,
    output                                      param_rd_rsp_ready,
    input              [511:0]                  param_bias_data,
    input              [255:0]                  param_mult_data,
    input              [63:0]                   param_shift_data,
    input              [63:0]                   param_zero_point_data,
    output                                      out_wr_valid,
    input                                       out_wr_ready,
    output             [ADDR_WIDTH-1:0]         out_wr_addr,
    output             [63:0]                   out_wr_data,
    output             [7:0]                    out_wr_strb
);

// FSM 状态编码。REQ 与 WAIT 分开，保证请求 valid 可以保持到 ready。
// ── 01 状态编码：请求、等待返回、计算、排空和写回分别建模 ──────────────
localparam S_IDLE         = 5'd0;
localparam S_CHECK        = 5'd1;
localparam S_PARAM_REQ    = 5'd2;
localparam S_PARAM_WAIT   = 5'd3;
localparam S_WEIGHT_REQ   = 5'd4;
localparam S_WEIGHT_WAIT  = 5'd5;
localparam S_WEIGHT_LOAD  = 5'd6;
localparam S_STREAM       = 5'd7;
localparam S_DRAIN        = 5'd8;
localparam S_WAIT_ACC     = 5'd9;
localparam S_POST_READ    = 5'd10;
localparam S_POST_WAIT    = 5'd11;
localparam S_POST_WRITE   = 5'd12;
localparam S_DONE         = 5'd13;
localparam S_ERROR        = 5'd14;
reg [4:0] state;

// start 仅在 IDLE 接受；该时钟沿同时锁存全部外部配置。
// ── 02 配置锁存：一层运算期间所有地址与尺寸均取锁存值 ────────────────
wire latch_en = (state == S_IDLE) && start;
wire [31:0] input_base;
wire [31:0] output_base;
wire [31:0] weight_base;
wire [8:0]  cin;
wire [8:0]  cout;
wire [8:0]  input_length;
wire [8:0]  output_length;
wire [4:0]  kernel;
wire [3:0]  stride;
wire [3:0]  dilation;
wire [7:0]  left_pad;
wire [11:0] k_total;
wire [9:0]  k_groups;
wire [5:0]  n_groups;
wire        elu_enable;
wire        relu_enable;
conv1d_config_latch u_cfg(
    .clk                         (clk),
    .rst_n                       (rst_n),
    .latch_en                    (latch_en),
    .cfg_input_base              (cfg_input_base),
    .cfg_output_base             (cfg_output_base),
    .cfg_weight_base             (cfg_weight_base),
    .cfg_cin                     (cfg_cin),
    .cfg_cout                    (cfg_cout),
    .cfg_kernel                  (cfg_kernel),
    .cfg_stride                  (cfg_stride),
    .cfg_dilation                (cfg_dilation),
    .cfg_left_pad                (cfg_left_pad),
    .cfg_input_length            (cfg_input_length),
    .cfg_output_length           (cfg_output_length),
    .cfg_k_total                 (cfg_k_total),
    .cfg_k_groups                (cfg_k_groups),
    .cfg_n_groups                (cfg_n_groups),
    .cfg_elu_enable              (cfg_elu_enable),
    .cfg_relu_enable             (cfg_relu_enable),
    .input_base                  (input_base),
    .output_base                 (output_base),
    .weight_base                 (weight_base),
    .cin                         (cin),
    .cout                        (cout),
    .kernel                      (kernel),
    .stride                      (stride),
    .dilation                    (dilation),
    .left_pad                    (left_pad),
    .input_length                (input_length),
    .output_length               (output_length),
    .k_total                     (k_total),
    .k_groups                    (k_groups),
    .n_groups                    (n_groups),
    .elu_enable                  (elu_enable),
    .relu_enable                 (relu_enable)
);

// output_group：当前 8 个输出通道的组号。
// k_group：当前 4 个展开 K 元素的组号。
// ── 03 循环与完成计数：输出通道组 / 规约组 / 时间位置 ─────────────────
reg [5:0] output_group;
reg [9:0] k_group;

// issued_count：本 K 组已被 Mesh 接受的 m 数量。
// received_count：本 K 组已被 Accumulator 接受的 m 数量。
// 两者都达到 output_length，才能认为当前 K 组真正排空。
reg [9:0] issued_count;
reg [9:0] received_count;

// 后处理阶段正在读取/写回的时间位置。
reg [M_TAG_WIDTH-1:0] post_m;

// 一个 output_group 的 8 路量化参数，在全部 M 个位置上复用。
reg [511:0] bias_q;
reg [255:0] mult_q;
reg [63:0] shift_q;
reg [63:0] zero_point_q;

// 配置合法性检查：除基本非零/范围检查外，还核对软件提供的派生量。
wire cin_ok = (cin == 9'd1)   || (cin == 9'd8)  ||
    (cin == 9'd16) ||
    (cin == 9'd32)  || (cin == 9'd64) ||
    (cin == 9'd128) || (cin == 9'd256);
wire [13:0] calc_k_total  = {5'd0, cin} * {9'd0, kernel};
wire [12:0] calc_k_groups = ({1'b0, k_total} + 13'd3) >> 2;
wire [8:0]  calc_n_groups = (cout + 9'd7) >> 3;
wire config_ok = cin_ok &&
    (cout != 0) &&
    (kernel != 0) &&
    (stride != 0) &&
    (dilation != 0) &&
    (input_length <= MAX_M) &&
    (output_length != 0) &&
    (output_length <= MAX_M) &&
    (k_total == calc_k_total[11:0]) &&
    (k_groups == calc_k_groups) &&
    (n_groups == calc_n_groups);
// 连续赋值：组合生成param_rd_req_valid及其相邻接口信号，表达握手、选择或地址关系。
assign param_rd_req_valid    = (state == S_PARAM_REQ);
assign param_rd_output_group = output_group;
// 连续赋值：组合生成param_rd_rsp_ready及其相邻接口信号，表达握手、选择或地址关系。
assign param_rd_rsp_ready    = (state == S_PARAM_WAIT);

// ── 04 激活通路：AGU产生地址，Reader管理请求、返回数据和m标签 ──────────
wire [ADDR_WIDTH-1:0] agu_a0;
wire [ADDR_WIDTH-1:0] agu_a1;
wire [ADDR_WIDTH-1:0] agu_a2;
wire [ADDR_WIDTH-1:0] agu_a3;
wire [3:0]            agu_mask;
wire                  agu_cin_valid;
conv1d_agu #(
    .ADDR_WIDTH                  (ADDR_WIDTH),
    .M_TAG_WIDTH                 (M_TAG_WIDTH)
) u_agu (
    .input_base                  (input_base),
    .cin                         (cin),
    .stride                      (stride),
    .dilation                    (dilation),
    .left_pad                    (left_pad),
    .input_length                (input_length),
    .k_total                     (k_total),
    .k_group                     (k_group),
    .m                           (issued_count[M_TAG_WIDTH-1:0]),
    .addr0                       (agu_a0),
    .addr1                       (agu_a1),
    .addr2                       (agu_a2),
    .addr3                       (agu_a3),
    .valid_mask                  (agu_mask),
    .cin_valid                   (agu_cin_valid)
);

wire                   ar_cmd_ready;
wire                   ar_rsp_valid;
wire                   ar_rsp_ready;
wire [63:0]            ar_rsp_data;
wire [M_TAG_WIDTH-1:0] ar_rsp_tag;

// issued_count 同时作为下一个 Activation 请求的 m_tag。
wire ar_cmd_valid = (state == S_STREAM) &&
    (issued_count < output_length);
activation_reader_4lane #(
    .ADDR_WIDTH                  (ADDR_WIDTH),
    .M_TAG_WIDTH                 (M_TAG_WIDTH)
) u_ar (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .cmd_valid                   (ar_cmd_valid),
    .cmd_ready                   (ar_cmd_ready),
    .cmd_addr0                   (agu_a0),
    .cmd_addr1                   (agu_a1),
    .cmd_addr2                   (agu_a2),
    .cmd_addr3                   (agu_a3),
    .cmd_mask                    (agu_mask),
    .cmd_m_tag                   (issued_count[M_TAG_WIDTH-1:0]),
    .act_rd_req_valid            (act_rd_req_valid),
    .act_rd_req_ready            (act_rd_req_ready),
    .act_rd_addr0                (act_rd_addr0),
    .act_rd_addr1                (act_rd_addr1),
    .act_rd_addr2                (act_rd_addr2),
    .act_rd_addr3                (act_rd_addr3),
    .act_rd_mask                 (act_rd_mask),
    .act_rd_rsp_valid            (act_rd_rsp_valid),
    .act_rd_rsp_ready            (act_rd_rsp_ready),
    .act_rd_data                 (act_rd_data),
    .rsp_valid                   (ar_rsp_valid),
    .rsp_ready                   (ar_rsp_ready),
    .rsp_data                    (ar_rsp_data),
    .rsp_m_tag                   (ar_rsp_tag)
);

// ── 05 权重通路：请求下一k_group并通过FIFO缓存返回的256-bit块 ───────────
wire         wl_cmd_ready;
wire         weight_valid;
wire         weight_ready;
wire [255:0] local_weight;
wire [31:0] output_group_ext = {{26{1'b0}}, output_group};
wire [31:0] k_groups_ext     = {{22{1'b0}}, k_groups};
wire [31:0] k_group_ext      = {{22{1'b0}}, k_group};
wire [31:0] next_k_group_ext = {{22{1'b0}}, k_group + 1'b1};

// 当前 k_group 计算时，提前请求下一块权重。weight_loader 内部的两项 FIFO
// 分别保存“待装载权重”和“预取权重”。
reg  next_weight_requested;
wire have_next_k_group = ((k_group + 1'b1) < k_groups);
wire weight_prefetch_phase = ((state == S_STREAM) ||
    (state == S_DRAIN)  ||
    (state == S_WAIT_ACC)) &&
    have_next_k_group &&
    !next_weight_requested;
wire wl_cmd_valid = (state == S_WEIGHT_REQ) || weight_prefetch_phase;

// 每个权重块固定 32 Byte：base + (og*K_groups + kg)*32。
wire [ADDR_WIDTH-1:0] weight_cmd_addr =
    weight_base + ((output_group_ext * k_groups_ext +
    (weight_prefetch_phase ? next_k_group_ext : k_group_ext)) << 5);
wire wl_cmd_fire = wl_cmd_valid && wl_cmd_ready;
weight_loader #(
    .ADDR_WIDTH                  (ADDR_WIDTH)
) u_wl (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .cmd_valid                   (wl_cmd_valid),
    .cmd_ready                   (wl_cmd_ready),
    .cmd_addr                    (weight_cmd_addr),
    .wgt_rd_req_valid            (wgt_rd_req_valid),
    .wgt_rd_req_ready            (wgt_rd_req_ready),
    .wgt_rd_addr                 (wgt_rd_addr),
    .wgt_rd_rsp_valid            (wgt_rd_rsp_valid),
    .wgt_rd_rsp_ready            (wgt_rd_rsp_ready),
    .wgt_rd_data                 (wgt_rd_data),
    .weight_valid                (weight_valid),
    .weight_ready                (weight_ready),
    .weight_data                 (local_weight)
);

// ── 06 阵列装载：分别屏蔽Active/Shadow对应的K尾部与Cout尾部 ────────────
reg [255:0] masked_weight;
reg [255:0] masked_next_weight;
integer wk;
integer wn;

// 组合逻辑：根据当前输入计算masked_weight、wk、wn、masked_next_weight、mesh_ce、ar_rsp_ready；本逻辑块不保存跨周期状态。
always @(*) begin

    // 即使软件预填零，RTL 仍再次屏蔽 K-tail 和 Cout-tail，形成硬件保护。
    masked_weight = 256'd0;
    for (wk = 0; wk < 4; wk = wk + 1) begin
        for (wn = 0; wn < 8; wn = wn + 1) begin
            if (((({2'b00, k_group} << 2) + wk) < k_total) &&
                ((({3'b000, output_group} << 3) + wn) < cout)) begin
                masked_weight[8*(wk*8+wn) +: 8] =
                    local_weight[8*(wk*8+wn) +: 8];
            end
        end
    end
end

// 组合逻辑：根据当前输入计算masked_next_weight、wk、wn、mesh_ce、ar_rsp_ready、mesh_direct_weight_load；本逻辑块不保存跨周期状态。
always @(*) begin

    // Shadow寄存器保存下一k_group，K-tail判断必须使用k_group+1。
    masked_next_weight = 256'd0;
    for (wk = 0; wk < 4; wk = wk + 1) begin
        for (wn = 0; wn < 8; wn = wn + 1) begin
            if (((({2'b00, (k_group + 1'b1)} << 2) + wk) < k_total) &&
                ((({3'b000, output_group} << 3) + wn) < cout)) begin
                masked_next_weight[8*(wk*8+wn) +: 8] =
                    local_weight[8*(wk*8+wn) +: 8];
            end
        end
    end
end

wire                   mesh_psum_valid;
wire [207:0]           mesh_psum_data;
wire [M_TAG_WIDTH-1:0] mesh_psum_tag;
wire                   acc_in_ready;
wire                   acc_pipeline_busy;

// Mesh 输出被阻塞时 ce=0，要求真实 Mesh 同时冻结 data/valid/tag。
wire mesh_ce = !mesh_psum_valid || acc_in_ready;

// Activation 只有在 Mesh 可以推进时才从 reader 中取走。
// 连续赋值：组合生成ar_rsp_ready及其相邻接口信号，表达握手、选择或地址关系。
assign ar_rsp_ready = ((state == S_STREAM) || (state == S_DRAIN)) &&
    mesh_ce;

// 权重只在专用加载状态、且 Mesh 能推进时装载一个周期。
// Active/Shadow两组Weight Register。当前组计算时把下一组提前送入Shadow；
// 当前组的Accumulator流水排空后，再用weight_swap单周期切换。
reg mesh_shadow_valid;
wire mesh_direct_weight_load = (state == S_WEIGHT_LOAD) &&
    weight_valid && mesh_ce;
wire mesh_shadow_weight_load = ((state == S_STREAM) ||
    (state == S_DRAIN)) &&
    next_weight_requested && weight_valid &&
    !mesh_shadow_valid && mesh_ce;
wire mesh_weight_swap = (state == S_WAIT_ACC) && !acc_pipeline_busy &&
    have_next_k_group && mesh_shadow_valid && mesh_ce;
// 连续赋值：组合生成weight_ready及其相邻接口信号，表达握手、选择或地址关系。
assign weight_ready = mesh_direct_weight_load || mesh_shadow_weight_load;

// V1：Mesh 自己的权重、Skew、PE、Valid/Tag、Deskew 使用同一门控时钟。
// Controller、Reader、权重搬运和 Accumulator 始终使用原始 clk。
// DRAIN 依照真实 acc_fire 计数结束，不能因为 act_valid=0 就提前停钟。
// WAIT_ACC 整段开钟，覆盖 Shadow -> Active 交换及末尾有效位清除。
wire mesh_phase_active = (state == S_WEIGHT_LOAD) ||
    (state == S_STREAM) || (state == S_DRAIN) || (state == S_WAIT_ACC);
wire mesh_clk_en = mesh_phase_active && mesh_ce;
wire mesh_gclk;
generate
    if (MESH_CLOCK_GATE_ENABLE) begin : gen_mesh_clock_gate
        mesh_icg u_mesh_icg (
            .clk_i(clk), .en_i(mesh_clk_en),
            .test_en_i(mesh_scan_test_en), .gclk_o(mesh_gclk)
        );
    end else begin : gen_mesh_clock_bypass
        assign mesh_gclk = clk;
    end
endgenerate

mesh_adapter #(
    .M_TAG_WIDTH                 (M_TAG_WIDTH)
) u_mesh (
    .clk                         (mesh_gclk),
    .rst_n                       (rst_n),
    .ce                          (mesh_ce),
    .weight_load                 (mesh_direct_weight_load),
    .weight_shadow_load          (mesh_shadow_weight_load),
    .weight_swap                 (mesh_weight_swap),
    .weight_data                 (mesh_shadow_weight_load ? masked_next_weight :
    masked_weight),
    .act_valid                   (ar_rsp_valid && ar_rsp_ready),
    .act_data                    (ar_rsp_data),
    .act_m_tag                   (ar_rsp_tag),
    .psum_valid                  (mesh_psum_valid),
    .psum_data                   (mesh_psum_data),
    .psum_m_tag                  (mesh_psum_tag)
);

wire         acc_rd_req_ready;
wire         acc_rd_rsp_valid;
wire         acc_rd_rsp_ready;
wire [319:0] acc_rd_rsp_data;
wire [M_TAG_WIDTH-1:0] acc_rd_rsp_addr;
wire stream_state = (state == S_STREAM) || (state == S_DRAIN);

// acc_fire 是“一个 psum 向量已真正进入 Accumulator”的唯一计数条件。
wire acc_fire = mesh_psum_valid && acc_in_ready && stream_state;

// ── 07 累加通路：消费对齐后的8路部分和，按m存储INT32累计结果 ───────────
accumulator_8lane #(
    .MAX_M                       (MAX_M),
    .M_TAG_WIDTH                 (M_TAG_WIDTH)
) u_acc (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .in_valid                    (mesh_psum_valid && stream_state),
    .in_ready                    (acc_in_ready),
    .first_k_group               (k_group == 0),
    .in_m_tag                    (mesh_psum_tag),
    .in_psum                     (mesh_psum_data),
    .rd_req_valid                ((state == S_POST_READ) &&
    (post_m < output_length)),
    .rd_req_ready                (acc_rd_req_ready),
    .rd_req_addr                 (post_m),
    .rd_rsp_valid                (acc_rd_rsp_valid),
    .rd_rsp_ready                (acc_rd_rsp_ready),
    .rd_rsp_data                 (acc_rd_rsp_data),
    .rd_rsp_addr                 (acc_rd_rsp_addr),
    .pipeline_busy               (acc_pipeline_busy)
);

wire        post_valid;
wire        post_ready;
wire [127:0] post_data;
wire        post_in_ready;
wire        elu_in_ready;
wire        elu_out_valid;
wire        elu_out_ready;
wire [127:0] elu_out_data;
wire        elu_lut_cfg_ready_raw;
wire post_stream_active = (state == S_POST_READ);

// Accumulator响应FIFO的320-bit读MUX后新增一级弹性寄存器。
// Data/Tag/Valid必须作为同一事务一起推进；后处理反压时三者同时保持。
// 空槽或本拍下游消费旧值时可接收新值，所以连续握手时仍支持II=1。
reg                         post_skid_valid;
reg [319:0]                 post_skid_data;
reg [M_TAG_WIDTH-1:0]       post_skid_tag;
wire post_skid_ready = !post_skid_valid || post_in_ready;

// requant输出和ELU输出的m_tag分别镜像对应数据级的ready条件。
reg                         post_tag_valid;
reg [M_TAG_WIDTH-1:0]       post_tag_q;
reg                         elu_tag_valid;
reg [M_TAG_WIDTH-1:0]       elu_tag_q;
reg [9:0]                   post_written_count;
// 连续赋值：组合生成acc_rd_rsp_ready及其相邻接口信号，表达握手、选择或地址关系。
assign acc_rd_rsp_ready = post_stream_active && post_skid_ready;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        post_skid_valid <= 1'b0;
        post_skid_data  <= 320'd0;
        post_skid_tag   <= {M_TAG_WIDTH{1'b0}};
    end else if (!post_stream_active) begin
        // 退出后处理阶段前，控制器已等待最后一个完整向量写回。
        // 此处清valid隔离不同输出通道组/层，数据寄存器无事务时保持。
        post_skid_valid <= 1'b0;
    end else if (post_skid_ready) begin
        post_skid_valid <= acc_rd_rsp_valid;
        if (acc_rd_rsp_valid) begin
            post_skid_data <= acc_rd_rsp_data;
            post_skid_tag  <= acc_rd_rsp_addr;
        end
    end
end

// ── 08 后处理：每个Cout独立Bias/Multiplier/Shift，产生INT8写回数据 ───────
postprocess_8lane u_post (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .in_valid                    (post_skid_valid && post_stream_active),
    .in_ready                    (post_in_ready),
    .acc_data                    (post_skid_data),
    .bias_data                   (bias_q),
    .mult_data                   (mult_q),
    .shift_data                  (shift_q),
    .zero_point_data             (zero_point_q),
    .post_valid                  (post_valid),
    .post_ready                  (post_ready),
    .post_data                   (post_data)
);

reg [7:0] write_strb;
reg [9:0] channels_left;

// 组合逻辑：根据当前输入计算channels_left、write_strb、post_m_ext、cout_ext、write_addr、post_ready；本逻辑块不保存跨周期状态。
always @(*) begin

    // 最后一个 output_group 可能不足 8 通道，按剩余 Cout 生成 byte strobe。
    channels_left = {1'b0, cout} - ({4'b0000, output_group} << 3);
    if (channels_left >= 10'd8) begin
        write_strb = 8'hff;
    end else if (channels_left == 0) begin
        write_strb = 8'h00;
    end else begin
        write_strb = (8'b00000001 << channels_left) - 8'b00000001;
    end
end

wire writer_in_ready;
wire writer_vector_done;
wire [31:0] post_m_ext = {{(32-M_TAG_WIDTH){1'b0}}, elu_tag_q};
wire [31:0] cout_ext   = {{23{1'b0}}, cout};

// 输出采用 time-major 布局：Y[m][channel]。
wire [ADDR_WIDTH-1:0] write_addr =
    output_base + ((post_m_ext * cout_ext + (output_group_ext << 3)) << 1);

// 后处理结果先进入 ELU。ELU 内部带 1 级弹性寄存器，因此 writer 反压会
// 沿 elu_out_ready -> elu_in_ready -> post_ready 逐级返回，不会覆盖数据。
// 连续赋值：组合生成post_ready及其相邻接口信号，表达握手、选择或地址关系。
assign post_ready         = post_stream_active && elu_in_ready;
assign elu_out_ready      = writer_in_ready;
// 连续赋值：组合生成elu_lut_cfg_ready及其相邻接口信号，表达握手、选择或地址关系。
assign elu_lut_cfg_ready  = (state == S_IDLE) && elu_lut_cfg_ready_raw;
elu_8lane u_elu (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .cfg_valid                   (elu_lut_cfg_valid && (state == S_IDLE)),
    .cfg_ready                   (elu_lut_cfg_ready_raw),
    .cfg_addr                    (elu_lut_cfg_addr),
    .cfg_wdata                   (elu_lut_cfg_wdata),
    .elu_enable                  (elu_enable),
    .relu_enable                 (relu_enable),
    .in_valid                    (post_valid && post_tag_valid && post_stream_active),
    .in_ready                    (elu_in_ready),
    .in_data                     (post_data),
    .out_valid                   (elu_out_valid),
    .out_ready                   (elu_out_ready),
    .out_data                    (elu_out_data)
);

output_writer_8lane #(
    .ADDR_WIDTH                  (ADDR_WIDTH)
) u_ow (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .in_valid                    (elu_out_valid && elu_tag_valid && post_stream_active),
    .in_ready                    (writer_in_ready),
    .in_addr                     (write_addr),
    .in_data                     (elu_out_data),
    .in_strb                     (write_strb),
    .out_wr_valid                (out_wr_valid),
    .out_wr_ready                (out_wr_ready),
    .out_wr_addr                 (out_wr_addr),
    .out_wr_data                 (out_wr_data),
    .out_wr_strb                 (out_wr_strb),
    .vector_done                 (writer_vector_done)
);

// skid已经锁存FIFO响应Tag；下面两级继续镜像postprocess与ELU。
// 禁止继续使用acc_rd_rsp_addr，否则增加一拍后数据和写回地址会错位。

// 时序逻辑：在时钟沿更新post_tag_valid、post_tag_q、elu_tag_valid、elu_tag_q、state、busy；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        post_tag_valid <= 1'b0;
        post_tag_q     <= {M_TAG_WIDTH{1'b0}};
        elu_tag_valid  <= 1'b0;
        elu_tag_q      <= {M_TAG_WIDTH{1'b0}};
    end else begin
        if (post_in_ready) begin
            post_tag_valid <= post_skid_valid && post_stream_active;
            if (post_skid_valid && post_stream_active)
                post_tag_q <= post_skid_tag;
        end
        if (elu_in_ready) begin
            elu_tag_valid <= post_valid && post_tag_valid &&
                post_stream_active;
            if (post_valid && post_tag_valid && post_stream_active)
                elu_tag_q <= post_tag_q;
        end
    end
end

// 时序逻辑：在时钟沿更新state、busy、done、error、output_group、k_group；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state          <= S_IDLE;
        busy           <= 1'b0;
        done           <= 1'b0;
        error          <= 1'b0;
        output_group   <= 6'd0;
        k_group        <= 10'd0;
        issued_count   <= 10'd0;
        received_count <= 10'd0;
        post_m         <= {M_TAG_WIDTH{1'b0}};
        post_written_count <= 10'd0;
        bias_q         <= 512'd0;
        mult_q         <= 256'd0;
        shift_q        <= 64'd0;
        zero_point_q   <= 64'd0;
        next_weight_requested <= 1'b0;
        mesh_shadow_valid <= 1'b0;
    end else begin

        // done/error 默认清零，所以正常情况下都只保持一个周期。
        done  <= 1'b0;
        error <= 1'b0;
        case (state)

            // 等待新任务。busy=0 时才会到达本状态。
            S_IDLE: begin
                if (start) begin
                    busy         <= 1'b1;
                    output_group <= 6'd0;
                    k_group      <= 10'd0;
                    state        <= S_CHECK;
                end
            end

            // 配置已经在上一时钟沿锁存，此处检查锁存后的稳定值。
            S_CHECK: begin
                if (config_ok) begin
                    state <= S_PARAM_REQ;
                end else begin
                    state <= S_ERROR;
                end
            end

            // 先读取当前 8 个输出通道的 bias/multiplier/shift。
            S_PARAM_REQ: begin
                if (param_rd_req_ready) begin
                    state <= S_PARAM_WAIT;
                end
            end
            S_PARAM_WAIT: begin
                if (param_rd_rsp_valid) begin
                    bias_q  <= param_bias_data;
                    mult_q  <= param_mult_data;
                    shift_q <= param_shift_data;
                    zero_point_q <= param_zero_point_data;
                    next_weight_requested <= 1'b0;
                    mesh_shadow_valid <= 1'b0;
                    state   <= S_WEIGHT_REQ;
                end
            end

            // 再读取当前 output_group、k_group 对应的 4x8 权重块。
            S_WEIGHT_REQ: begin
                if (wl_cmd_fire) begin
                    state <= S_WEIGHT_WAIT;
                end
            end
            S_WEIGHT_WAIT: begin
                if (weight_valid) begin
                    state <= S_WEIGHT_LOAD;
                end
            end
            S_WEIGHT_LOAD: begin
                if (weight_valid && weight_ready) begin
                    issued_count   <= 10'd0;
                    received_count <= 10'd0;
                    next_weight_requested <= 1'b0;
                    mesh_shadow_valid <= 1'b0;
                    state          <= S_STREAM;
                end
            end

            // 连续处理 m。issued_count 只在 Activation 被 Mesh 接受时增加。
            S_STREAM: begin

                // issued_count 统计“已被 SRAM 接收”的地址命令，而不是 Mesh
                // 已消费响应数；因此流水 Reader 可以连续产生不同 m 的地址。
                if (ar_cmd_valid && ar_cmd_ready) begin
                    issued_count <= issued_count + 1'b1;
                    if ((issued_count + 1'b1) >= output_length) begin
                        state <= S_DRAIN;
                    end
                end
                if (acc_fire) begin
                    received_count <= received_count + 1'b1;
                end
            end

            // 所有输入已经发出，但仍需按 received_count 等待在途结果返回。
            S_DRAIN: begin
                if (acc_fire) begin
                    received_count <= received_count + 1'b1;
                    if ((received_count + 1'b1) >= output_length) begin
                        state <= S_WAIT_ACC;
                    end
                end
            end

            // 最后一个 psum 可能刚进入 RMW 管线，所以还要等待写回完成。
            S_WAIT_ACC: begin
                if (!acc_pipeline_busy) begin
                    if ((k_group + 1'b1) < k_groups) begin
                        k_group <= k_group + 1'b1;

                        // 预取完成时直接进入装载；仍在返回途中时只等待响应。
                        // 极短层若尚未成功发出请求，则退回普通请求状态。
                        if (mesh_shadow_valid) begin
                            issued_count          <= 10'd0;
                            received_count        <= 10'd0;
                            next_weight_requested <= 1'b0;
                            mesh_shadow_valid     <= 1'b0;
                            state                 <= S_STREAM;
                        end else if (weight_valid)
                        state <= S_WEIGHT_LOAD;
                        else if (next_weight_requested ||
                            (weight_prefetch_phase && wl_cmd_fire))
                        state <= S_WEIGHT_WAIT;
                        else
                        state <= S_WEIGHT_REQ;
                    end else begin
                        post_m <= {M_TAG_WIDTH{1'b0}};
                        post_written_count <= 10'd0;
                        state  <= S_POST_READ;
                    end
                end
            end

            // 全部 K 组结束后逐 m 读取 Accumulator，并送入后处理流水。
            S_POST_READ: begin
                if ((post_m < output_length) && acc_rd_req_ready)
                    post_m <= post_m + 1'b1;
                // Writer会把8×INT16拆成两个64-bit写拍，只能在整个向量
                // 的最后一拍完成后递增计数，否则会提前切换output_group。
                if (writer_vector_done) begin
                    post_written_count <= post_written_count + 1'b1;
                    if ((post_written_count + 1'b1) >= output_length) begin
                        if ((output_group + 1'b1) < n_groups) begin
                            output_group <= output_group + 1'b1;
                            k_group      <= 10'd0;
                            state        <= S_PARAM_REQ;
                        end else begin
                            state <= S_DONE;
                        end
                    end
                end
            end
            S_POST_WAIT: begin
                state <= S_ERROR;
            end

            // 只有 Output SRAM 真正接受写请求后，才递增 m/output_group。
            S_POST_WRITE: begin
                state <= S_ERROR;
            end

            // done/error 都保持一个周期，随后返回 IDLE 并清 busy。
            S_DONE: begin
                done  <= 1'b1;
                busy  <= 1'b0;
                state <= S_IDLE;
            end
            S_ERROR: begin
                error <= 1'b1;
                busy  <= 1'b0;
                state <= S_IDLE;
            end
            default: begin
                state <= S_ERROR;
            end
        endcase
        if (weight_prefetch_phase && wl_cmd_fire)
            next_weight_requested <= 1'b1;
        if (mesh_shadow_weight_load)
            mesh_shadow_valid <= 1'b1;
    end
end

endmodule
