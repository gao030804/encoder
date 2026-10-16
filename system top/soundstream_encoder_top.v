// ============================================================================
// 中文阅读导引（当前实现）
// 帧级集成：调度器产生本层配置，计算引擎读取Activation和Weight，结果经残差通路写入另一组Activation SRAM。
// 层完成后交换输入/输出Buffer角色。参数存储仍通过顶层端口连接，RVQ尚未在本模块实例化。
// USE_GLOBAL_WEIGHT_HIERARCHY默认关闭；启用前必须更新紧凑模型的层表和权重镜像。
// ============================================================================
//=============================================================================
// 模块：soundstream_encoder_top
// 功能：31物理层SoundStream Conv1d编码器顶层。
// 包含自动层调度、Activation Ping-Pong、Residual Scratch/Add和Flash Weight A/B。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - Encoder结构顶层，负责Scheduler、Activation Ping-Pong、Residual Scratch和Weight Hierarchy之间的连接。
// - current_buffer_select指示本层输入Buffer，下一组Buffer接收本层输出；每个物理卷积完成后交换角色。
// - Activation读口在Residual Capture、卷积引擎和结果读取之间仲裁；运行时Capture优先级最高。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：SoundStream Encoder集成顶层。
// 关键变量/接口：连接63层调度、激活Ping-Pong、残差通路、卷积引擎和多级权重存储。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module soundstream_encoder_top #(
    parameter ADDR_WIDTH = 32,
    parameter BUFFER_A_BASE = 32'h0000_1000,
    parameter BUFFER_B_BASE = 32'h0000_5000,

    // 保留旧参数名以兼容现有testbench；Global路径中含义为QSPI半周期分频值。
    // QSPI半周期分频值取1时，Flash SCLK为主时钟的1/2。
    parameter FLASH_SPI_DIVIDER = 16'd1,

    // 置1后采用Flash->256 KiB Global->2x8 KiB Local的Encoder权重层级。
    // FAST_SIM_WEIGHT_CACHE仍具有更高优先级，便于已有全帧testbench快速运行。
    // 当前63层ROM使用162688 Byte packed weight，默认256 KiB Global SRAM可完整容纳。
    // 使用最新紧凑Encoder权重镜像和层描述表时将该参数置1。
    parameter USE_GLOBAL_WEIGHT_HIERARCHY = 1,
    parameter GLOBAL_DEPTH_BLOCKS = 8192,
    parameter GLOBAL_INDEX_WIDTH = 13,
    parameter GLOBAL_MODEL_BLOCKS = 5084,
    parameter BOOT_AUX_BLOCKS = 0,
    parameter BOOT_FLASH_BASE = 24'h000000,
    parameter USE_OCTAL_STREAM_WEIGHT_CACHE = 0,
    parameter FAST_SIM_WEIGHT_CACHE = 0,
    parameter RESIDUAL_SCALE_ALIGN_ENABLE = 0
) (
    output boot_aux_write,
    output [15:0] boot_aux_index,
    output [255:0] boot_aux_data,
    output boot_param_write,
    output [12:0] boot_param_block,
    output [255:0] boot_param_data,
    output boot_ready,
    input                                       clk,
    input                                       rst_n,
    input tri0                                  mesh_scan_test_en,
    input                                       start,
    output                                      start_ready,
    output                                      busy,
    output                                      done,
    output                                      error,

    // 编码前由PCM frontend向当前输入Buffer写入320个INT8样本。
    input                                       input_wr_valid,
    output                                      input_wr_ready,
    input              [ADDR_WIDTH-1:0]         input_wr_addr,
    input              [63:0]                   input_wr_data,
    input              [7:0]                    input_wr_strb,
    output                                      input_buffer_select,
    output             [ADDR_WIDTH-1:0]         input_buffer_base,

    // 每层开始前的准备握手；外部可在此阶段更新该层ELU LUT。
    output                                      layer_prepare_valid,
    input                                       layer_prepare_ready,
    // 与layer_prepare握手同步提交，当前层执行期间保持在内部寄存器中。
    input              [119:0]                  residual_scale_params,
    output             [5:0]                    physical_layer,
    output             [5:0]                    logical_layer,
    input                                       elu_lut_cfg_valid,
    output                                      elu_lut_cfg_ready,
    input              [6:0]                    elu_lut_cfg_addr,
    input              [7:0]                    elu_lut_cfg_wdata,

    // Parameter Memory用physical_layer和output_group联合寻址。
    output                                      param_rd_req_valid,
    input                                       param_rd_req_ready,
    output             [5:0]                    param_rd_layer,
    output             [5:0]                    param_rd_output_group,
    input                                       param_rd_rsp_valid,
    output                                      param_rd_rsp_ready,
    input              [511:0]                  param_bias_data,
    input              [255:0]                  param_mult_data,
    input              [63:0]                   param_shift_data,
    input              [63:0]                   param_zero_point_data,

    // 编码完成后读取最终1x64 INT8 latent，也可用于testbench读中间Buffer。
    input                                       result_rd_req_valid,
    output                                      result_rd_req_ready,
    input              [ADDR_WIDTH-1:0]         result_rd_addr0,
    input              [ADDR_WIDTH-1:0]         result_rd_addr1,
    input              [ADDR_WIDTH-1:0]         result_rd_addr2,
    input              [ADDR_WIDTH-1:0]         result_rd_addr3,
    input              [3:0]                    result_rd_mask,
    output                                      result_rd_rsp_valid,
    input                                       result_rd_rsp_ready,
    output             [63:0]                   result_rd_data,
    output                                      result_buffer_select,
    output             [ADDR_WIDTH-1:0]         result_buffer_base,

    // 逐层写回监视口，不参与反压，供验证和性能统计。
    output                                      layer_write_fire,
    output             [ADDR_WIDTH-1:0]         layer_write_addr,
    output             [63:0]                   layer_write_data,
    output             [7:0]                    layer_write_strb,

    // 性能计数观测口：不参与控制，仅供testbench/片上性能计数器使用。
    output                                      perf_engine_start,
    output                                      perf_engine_done,
    output                                      perf_weight_sram_read_fire,
    output                                      perf_weight_prefetch_fire,
    output                                      perf_weight_prefetch_done,
    output             [23:0]                   perf_weight_prefetch_base,
    output             [15:0]                   perf_weight_prefetch_blocks,
    output                                      flash_csn,
    output                                      flash_sclk,
    inout                                       flash_mosi,
    inout                                       flash_miso,
    inout                                       flash_wp,
    inout                                       flash_hold,

    // S28 OPI控制器/PHY完成DDR采样和CDC之后的命令/16-bit数据流接口。
    output                                      octal_cmd_valid,
    input                                       octal_cmd_ready,
    output             [23:0]                   octal_cmd_flash_base,
    output             [23:0]                   octal_cmd_byte_count,
    input                                       octal_phy_clk,
    input                                       octal_phy_rst_n,
    input                                       octal_rx_valid,
    output                                      octal_rx_ready,
    input              [15:0]                   octal_rx_data,
    input                                       octal_rx_last,
    input                                       octal_rx_error
);

wire sched_error, engine_error, flash_error;
wire capture_error, add_error, scratch_error, buffer_access_error;
wire datapath_aux_error = capture_error | add_error |
    scratch_error | buffer_access_error;
wire [8:0] cfg_cin,cfg_cout,cfg_input_length,cfg_output_length;
wire [4:0] cfg_kernel;
wire [3:0] cfg_stride,cfg_dilation;
wire [7:0] cfg_left_pad;
wire [11:0] cfg_k_total;
wire [9:0] cfg_k_groups;
wire [5:0] cfg_n_groups;
wire cfg_is_depthwise, cfg_relu_enable, cfg_elu_enable, residual_add_enable;
wire [23:0] cfg_weight_base24;
wire [15:0] cfg_weight_blocks;
wire current_buffer_select;
wire weight_prefetch_valid,weight_prefetch_ready,weight_prefetch_done;
wire [23:0] weight_prefetch_base;
wire [15:0] weight_prefetch_blocks;
wire weight_prefetch_bank,weight_activate_valid,weight_activate_ready;
wire weight_activate_first,weight_activate_bank;
wire first_weight_ready,active_weight_first,active_weight_bank,active_weight_valid;
assign boot_ready = first_weight_ready;
wire flash_busy;
wire engine_start,engine_busy,engine_done;
wire engine_weight_read_fire;
wire capture_start,capture_start_ready,capture_done,capture_busy;
wire add_busy;
reg residual_done_pending;
wire scheduler_engine_done = residual_add_enable ?
    (residual_done_pending && !add_busy) :
    engine_done;

// engine_done只表示卷积引擎最后一个输出已被下游接受。
// Residual层还必须等待最后一个Scratch读、加法和Activation写回完成。

// 时序逻辑：在时钟沿更新residual_done_pending；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        residual_done_pending <= 1'b0;
    else begin
        if (engine_done && residual_add_enable)
            residual_done_pending <= 1'b1;
        else if (residual_done_pending && !add_busy)
            residual_done_pending <= 1'b0;
    end
end

soundstream_encoder_scheduler u_scheduler (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .start                       (start),
    .start_ready                 (start_ready),
    .busy                        (busy),
    .done                        (done),
    .error                       (sched_error),
    .first_layer_weight_ready    (first_weight_ready),
    .active_weight_is_first_layer (active_weight_first),
    .weight_error                (flash_error),
    .weight_prefetch_valid       (weight_prefetch_valid),
    .weight_prefetch_ready       (weight_prefetch_ready),
    .weight_prefetch_flash_base  (weight_prefetch_base),
    .weight_prefetch_block_count (weight_prefetch_blocks),
    .weight_prefetch_target_bank (weight_prefetch_bank),
    .weight_prefetch_done        (weight_prefetch_done),
    .weight_activate_valid       (weight_activate_valid),
    .weight_activate_ready       (weight_activate_ready),
    .weight_activate_first_layer (weight_activate_first),
    .weight_activate_bank        (weight_activate_bank),
    .engine_start                (engine_start),
    .engine_done                 (scheduler_engine_done),
    .engine_error                (engine_error),
    .capture_start               (capture_start),
    .capture_start_ready         (capture_start_ready),
    .capture_done                (capture_done),
    .capture_error               (datapath_aux_error),
    .layer_prepare_valid         (layer_prepare_valid),
    .layer_prepare_ready         (layer_prepare_ready),
    .physical_layer              (physical_layer),
    .logical_layer               (logical_layer),
    .current_buffer_select       (current_buffer_select),
    .residual_add_enable         (residual_add_enable),
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
    .cfg_is_depthwise            (cfg_is_depthwise),
    .cfg_relu_enable             (cfg_relu_enable),
    .cfg_elu_enable              (cfg_elu_enable),
    .cfg_weight_flash_base       (cfg_weight_base24),
    .cfg_weight_block_count      (cfg_weight_blocks)
);

// 连续赋值：组合生成error及其相邻接口信号，表达握手、选择或地址关系。
assign error = sched_error | engine_error | flash_error | datapath_aux_error;
assign input_buffer_select = current_buffer_select;
// 连续赋值：组合生成input_buffer_base及其相邻接口信号，表达握手、选择或地址关系。
assign input_buffer_base = current_buffer_select ? BUFFER_B_BASE : BUFFER_A_BASE;
assign result_buffer_select = current_buffer_select;
// 连续赋值：组合生成result_buffer_base及其相邻接口信号，表达握手、选择或地址关系。
assign result_buffer_base = input_buffer_base;
assign param_rd_layer = physical_layer;
wire [ADDR_WIDTH-1:0] current_base = current_buffer_select ? BUFFER_B_BASE : BUFFER_A_BASE;
wire [ADDR_WIDTH-1:0] next_base = current_buffer_select ? BUFFER_A_BASE : BUFFER_B_BASE;
wire eng_act_req_valid,eng_act_req_ready,eng_act_rsp_valid,eng_act_rsp_ready;
wire [ADDR_WIDTH-1:0] eng_act_addr0,eng_act_addr1,eng_act_addr2,eng_act_addr3;
wire [3:0] eng_act_mask;
wire [63:0] eng_act_data;
wire eng_out_valid,eng_out_ready;
wire [ADDR_WIDTH-1:0] eng_out_addr;
wire [63:0] eng_out_data;
wire [7:0] eng_out_strb;
conv1d_engine_flash_top #(
    .FLASH_SPI_DIVIDER           (FLASH_SPI_DIVIDER),
    .WEIGHT_CACHE_BLOCKS         (4096),
    .WEIGHT_CACHE_INDEX_WIDTH    (12),
    .FIRST_LAYER_FLASH_BASE      (24'h000000),
    .USE_GLOBAL_WEIGHT_HIERARCHY (USE_GLOBAL_WEIGHT_HIERARCHY),
    .GLOBAL_DEPTH_BLOCKS         (GLOBAL_DEPTH_BLOCKS),
    .GLOBAL_INDEX_WIDTH          (GLOBAL_INDEX_WIDTH),
    .GLOBAL_MODEL_BLOCKS         (GLOBAL_MODEL_BLOCKS),
    .BOOT_AUX_BLOCKS             (BOOT_AUX_BLOCKS),
    .BOOT_FLASH_BASE             (BOOT_FLASH_BASE),
    .USE_OCTAL_STREAM_CACHE      (USE_OCTAL_STREAM_WEIGHT_CACHE),
    .FAST_SIM_WEIGHT_CACHE       (FAST_SIM_WEIGHT_CACHE)
) u_engine_flash (
    .mesh_scan_test_en            (mesh_scan_test_en),
    .boot_aux_write(boot_aux_write),.boot_aux_index(boot_aux_index),.boot_aux_data(boot_aux_data),
    .boot_param_write(boot_param_write),.boot_param_block(boot_param_block),
    .boot_param_data(boot_param_data),
    .clk                         (clk),
    .rst_n                       (rst_n),
    .start                       (engine_start),
    .busy                        (engine_busy),
    .done                        (engine_done),
    .error                       (engine_error),
    .flash_busy                  (flash_busy),
    .flash_error                 (flash_error),
    .first_layer_weight_ready    (first_weight_ready),
    .active_weight_is_first_layer (active_weight_first),
    .active_weight_bank          (active_weight_bank),
    .active_weight_bank_valid    (active_weight_valid),
    .perf_weight_read_fire       (engine_weight_read_fire),
    .weight_prefetch_valid       (weight_prefetch_valid),
    .weight_prefetch_ready       (weight_prefetch_ready),
    .weight_prefetch_flash_base  (weight_prefetch_base),
    .weight_prefetch_block_count (weight_prefetch_blocks),
    .weight_prefetch_target_bank (weight_prefetch_bank),
    .weight_prefetch_done        (weight_prefetch_done),
    .weight_activate_valid       (weight_activate_valid),
    .weight_activate_ready       (weight_activate_ready),
    .weight_activate_first_layer (weight_activate_first),
    .weight_activate_bank        (weight_activate_bank),
    .cfg_input_base              (current_base),
    .cfg_output_base             (next_base),
    .cfg_weight_base             ({8'd0,cfg_weight_base24}),
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
    .cfg_is_depthwise            (cfg_is_depthwise),
    .cfg_relu_enable             (cfg_relu_enable),
    .cfg_elu_enable              (cfg_elu_enable),
    .elu_lut_cfg_valid           (elu_lut_cfg_valid),
    .elu_lut_cfg_ready           (elu_lut_cfg_ready),
    .elu_lut_cfg_addr            (elu_lut_cfg_addr),
    .elu_lut_cfg_wdata           (elu_lut_cfg_wdata),
    .act_rd_req_valid            (eng_act_req_valid),
    .act_rd_req_ready            (eng_act_req_ready),
    .act_rd_addr0                (eng_act_addr0),
    .act_rd_addr1                (eng_act_addr1),
    .act_rd_addr2                (eng_act_addr2),
    .act_rd_addr3                (eng_act_addr3),
    .act_rd_mask                 (eng_act_mask),
    .act_rd_rsp_valid            (eng_act_rsp_valid),
    .act_rd_rsp_ready            (eng_act_rsp_ready),
    .act_rd_data                 (eng_act_data),
    .param_rd_req_valid          (param_rd_req_valid),
    .param_rd_req_ready          (param_rd_req_ready),
    .param_rd_output_group       (param_rd_output_group),
    .param_rd_rsp_valid          (param_rd_rsp_valid),
    .param_rd_rsp_ready          (param_rd_rsp_ready),
    .param_bias_data             (param_bias_data),
    .param_mult_data             (param_mult_data),
    .param_shift_data            (param_shift_data),
    .param_zero_point_data       (param_zero_point_data),
    .out_wr_valid                (eng_out_valid),
    .out_wr_ready                (eng_out_ready),
    .out_wr_addr                 (eng_out_addr),
    .out_wr_data                 (eng_out_data),
    .out_wr_strb                 (eng_out_strb),
    .flash_csn                   (flash_csn),
    .flash_sclk                  (flash_sclk),
    .flash_mosi                  (flash_mosi),
    .flash_miso                  (flash_miso),
    .flash_wp                    (flash_wp),
    .flash_hold                  (flash_hold),
    .octal_cmd_valid             (octal_cmd_valid),
    .octal_cmd_ready             (octal_cmd_ready),
    .octal_cmd_flash_base        (octal_cmd_flash_base),
    .octal_cmd_byte_count        (octal_cmd_byte_count),
    .octal_phy_clk               (octal_phy_clk),
    .octal_phy_rst_n             (octal_phy_rst_n),
    .octal_rx_valid              (octal_rx_valid),
    .octal_rx_ready              (octal_rx_ready),
    .octal_rx_data               (octal_rx_data),
    .octal_rx_last               (octal_rx_last),
    .octal_rx_error              (octal_rx_error)
);

// Residual capture读取当前Buffer并写入8 KiB Scratch。
wire cap_act_req_valid,cap_act_req_ready,cap_act_rsp_valid,cap_act_rsp_ready;
wire [ADDR_WIDTH-1:0] cap_addr0,cap_addr1,cap_addr2,cap_addr3;
wire [3:0] cap_mask;
wire [63:0] cap_data;
wire scratch_wr_valid,scratch_wr_ready;
wire [10:0] scratch_wr_addr;
wire [63:0] scratch_wr_data;
wire [7:0] scratch_wr_strb;
wire [17:0] capture_bytes_full = (cfg_cin * cfg_input_length) << 1;
residual_capture u_capture (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .start                       (capture_start),
    .start_ready                 (capture_start_ready),
    .input_base                  (current_base),
    .byte_count                  (capture_bytes_full[13:0]),
    .busy                        (capture_busy),
    .done                        (capture_done),
    .error                       (capture_error),
    .act_req_valid               (cap_act_req_valid),
    .act_req_ready               (cap_act_req_ready),
    .act_addr0                   (cap_addr0),
    .act_addr1                   (cap_addr1),
    .act_addr2                   (cap_addr2),
    .act_addr3                   (cap_addr3),
    .act_mask                    (cap_mask),
    .act_rsp_valid               (cap_act_rsp_valid),
    .act_rsp_ready               (cap_act_rsp_ready),
    .act_rsp_data                (cap_data),
    .scratch_wr_valid            (scratch_wr_valid),
    .scratch_wr_ready            (scratch_wr_ready),
    .scratch_wr_addr             (scratch_wr_addr),
    .scratch_wr_data             (scratch_wr_data),
    .scratch_wr_strb             (scratch_wr_strb)
);

wire add_in_ready,add_out_valid,add_out_ready;
wire [ADDR_WIDTH-1:0] add_out_addr;
wire [63:0] add_out_data;
wire [7:0] add_out_strb;
wire scratch_rd_valid,scratch_rd_ready,scratch_rsp_valid;
wire [10:0] scratch_rd_addr;
wire [63:0] scratch_rsp_data;
reg [119:0] residual_params_q;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) residual_params_q <= 120'd0;
    else if (layer_prepare_valid && layer_prepare_ready)
        residual_params_q <= residual_scale_params;
end

residual_add_8lane #(.SCALE_ALIGN_ENABLE(RESIDUAL_SCALE_ALIGN_ENABLE)) u_residual_add (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .in_valid                    (eng_out_valid && residual_add_enable),
    .in_ready                    (add_in_ready),
    .busy                        (add_busy),
    .in_addr                     (eng_out_addr),
    .output_base                 (next_base),
    .in_data                     (eng_out_data),
    .in_strb                     (eng_out_strb),
    .scale_params                (residual_params_q),
    .scratch_rd_valid            (scratch_rd_valid),
    .scratch_rd_ready            (scratch_rd_ready),
    .scratch_rd_addr             (scratch_rd_addr),
    .scratch_rsp_valid           (scratch_rsp_valid),
    .scratch_rsp_data            (scratch_rsp_data),
    .out_valid                   (add_out_valid),
    .out_ready                   (add_out_ready),
    .out_addr                    (add_out_addr),
    .out_data                    (add_out_data),
    .out_strb                    (add_out_strb),
    .error                       (add_error)
);

residual_scratchpad u_residual_scratch (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .wr_valid                    (scratch_wr_valid),
    .wr_ready                    (scratch_wr_ready),
    .wr_addr                     (scratch_wr_addr),
    .wr_data                     (scratch_wr_data),
    .wr_strb                     (scratch_wr_strb),
    .rd_valid                    (scratch_rd_valid),
    .rd_ready                    (scratch_rd_ready),
    .rd_addr                     (scratch_rd_addr),
    .rsp_valid                   (scratch_rsp_valid),
    .rsp_data                    (scratch_rsp_data),
    .error                       (scratch_error)
);

wire encoded_wr_valid = residual_add_enable ? add_out_valid : eng_out_valid;
wire [ADDR_WIDTH-1:0] encoded_wr_addr = residual_add_enable ? add_out_addr : eng_out_addr;
wire [63:0] encoded_wr_data = residual_add_enable ? add_out_data : eng_out_data;
wire [7:0] encoded_wr_strb = residual_add_enable ? add_out_strb : eng_out_strb;
wire buffer_wr_ready;
// 连续赋值：组合生成eng_out_ready及其相邻接口信号，表达握手、选择或地址关系。
assign eng_out_ready = residual_add_enable ? add_in_ready : buffer_wr_ready;
assign add_out_ready = buffer_wr_ready;

// Activation Buffer读端口：capture优先；编码忙时归engine；空闲时供结果读取。
wire buf_rd_req_valid,buf_rd_req_ready,buf_rd_rsp_valid,buf_rd_rsp_ready;
wire [ADDR_WIDTH-1:0] buf_addr0,buf_addr1,buf_addr2,buf_addr3;
wire [3:0] buf_mask;
wire [63:0] buf_rsp_data;
// 连续赋值：组合生成buf_rd_req_valid及其相邻接口信号，表达握手、选择或地址关系。
assign buf_rd_req_valid = capture_busy ? cap_act_req_valid : (busy ? eng_act_req_valid : result_rd_req_valid);
assign buf_addr0 = capture_busy ? cap_addr0 : (busy ? eng_act_addr0 : result_rd_addr0);
// 连续赋值：组合生成buf_addr1及其相邻接口信号，表达握手、选择或地址关系。
assign buf_addr1 = capture_busy ? cap_addr1 : (busy ? eng_act_addr1 : result_rd_addr1);
assign buf_addr2 = capture_busy ? cap_addr2 : (busy ? eng_act_addr2 : result_rd_addr2);
// 连续赋值：组合生成buf_addr3及其相邻接口信号，表达握手、选择或地址关系。
assign buf_addr3 = capture_busy ? cap_addr3 : (busy ? eng_act_addr3 : result_rd_addr3);
assign buf_mask  = capture_busy ? cap_mask  : (busy ? eng_act_mask  : result_rd_mask);
// 连续赋值：组合生成cap_act_req_ready及其相邻接口信号，表达握手、选择或地址关系。
assign cap_act_req_ready = capture_busy && buf_rd_req_ready;
assign eng_act_req_ready = !capture_busy && busy && buf_rd_req_ready;
// 连续赋值：组合生成result_rd_req_ready及其相邻接口信号，表达握手、选择或地址关系。
assign result_rd_req_ready = !busy && buf_rd_req_ready;
assign cap_act_rsp_valid = capture_busy && buf_rd_rsp_valid;
// 连续赋值：组合生成eng_act_rsp_valid及其相邻接口信号，表达握手、选择或地址关系。
assign eng_act_rsp_valid = !capture_busy && busy && buf_rd_rsp_valid;
assign result_rd_rsp_valid = !busy && buf_rd_rsp_valid;
// 连续赋值：组合生成cap_data及其相邻接口信号，表达握手、选择或地址关系。
assign cap_data = buf_rsp_data;
assign eng_act_data = buf_rsp_data;
// 连续赋值：组合生成result_rd_data及其相邻接口信号，表达握手、选择或地址关系。
assign result_rd_data = buf_rsp_data;
assign buf_rd_rsp_ready = capture_busy ? cap_act_rsp_ready : (busy ? eng_act_rsp_ready : result_rd_rsp_ready);
wire buffer_wr_valid = busy ? encoded_wr_valid : input_wr_valid;
wire [ADDR_WIDTH-1:0] buffer_wr_addr = busy ? encoded_wr_addr : input_wr_addr;
wire [63:0] buffer_wr_data = busy ? encoded_wr_data : input_wr_data;
wire [7:0] buffer_wr_strb = busy ? encoded_wr_strb : input_wr_strb;
// 连续赋值：组合生成input_wr_ready及其相邻接口信号，表达握手、选择或地址关系。
assign input_wr_ready = !busy && buffer_wr_ready;
assign layer_write_fire = busy && encoded_wr_valid && buffer_wr_ready;
// 连续赋值：组合生成layer_write_addr及其相邻接口信号，表达握手、选择或地址关系。
assign layer_write_addr = encoded_wr_addr;
assign layer_write_data = encoded_wr_data;
// 连续赋值：组合生成layer_write_strb及其相邻接口信号，表达握手、选择或地址关系。
assign layer_write_strb = encoded_wr_strb;
assign perf_engine_start = engine_start;
// 连续赋值：组合生成perf_engine_done及其相邻接口信号，表达握手、选择或地址关系。
assign perf_engine_done = scheduler_engine_done;
assign perf_weight_sram_read_fire = engine_weight_read_fire;
// 连续赋值：组合生成perf_weight_prefetch_fire及其相邻接口信号，表达握手、选择或地址关系。
assign perf_weight_prefetch_fire = weight_prefetch_valid && weight_prefetch_ready;
assign perf_weight_prefetch_done = weight_prefetch_done;
// 连续赋值：组合生成perf_weight_prefetch_base及其相邻接口信号，表达握手、选择或地址关系。
assign perf_weight_prefetch_base = weight_prefetch_base;
assign perf_weight_prefetch_blocks = weight_prefetch_blocks;
activation_buffer_pingpong u_activation_buffer (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .rd_buffer_select            (current_buffer_select),
    .rd_buffer_base              (current_base),
    .rd_req_valid                (buf_rd_req_valid),
    .rd_req_ready                (buf_rd_req_ready),
    .rd_addr0                    (buf_addr0),
    .rd_addr1                    (buf_addr1),
    .rd_addr2                    (buf_addr2),
    .rd_addr3                    (buf_addr3),
    .rd_mask                     (buf_mask),
    .rd_rsp_valid                (buf_rd_rsp_valid),
    .rd_rsp_ready                (buf_rd_rsp_ready),
    .rd_rsp_data                 (buf_rsp_data),
    .wr_buffer_select            (busy ? ~current_buffer_select : current_buffer_select),
    .wr_buffer_base              (busy ? next_base : current_base),
    .wr_valid                    (buffer_wr_valid),
    .wr_ready                    (buffer_wr_ready),
    .wr_addr                     (buffer_wr_addr),
    .wr_data                     (buffer_wr_data),
    .wr_strb                     (buffer_wr_strb),
    .access_error                (buffer_access_error)
);

endmodule
