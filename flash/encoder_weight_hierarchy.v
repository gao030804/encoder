// ============================================================================
// 中文阅读导引（当前实现）
// 阅读分区：配置/端口 → Boot DMA与Global SRAM → Local地址命中/Tile选择 → A/B存储 → 握手 → 主状态机。
// GLOBAL_MODEL_BLOCKS以32 Byte为单位；当前5084块对应63层卷积权重，不包含RVQ码本与后处理参数。
// 当前Global是单个逻辑SRAM，Flash只停止读事务，尚未发送Deep Power-Down命令。
// Local miss在需求到达后触发搬运；现有逻辑不是完整的下一Tile提前预取流水。
// ============================================================================
//=============================================================================
// 模块名称：encoder_weight_hierarchy
// 功能：仅面向 Encoder 的四级权重存储层次。
//
//   外部 Flash
//       -> 256 KiB Global Weight SRAM（上电连续 Burst 装载一次）
//       -> 8 KiB Local Weight SRAM A/B（按层或按 Tile 交替预取）
//       -> weight_loader / Active-Shadow Weight Register
//       -> 4x8 Systolic Array
//
// 说明：
//   1. 本模块保持旧 weight_cache_ab 的请求/激活接口，便于接入现有控制器；
//   2. prefetch_flash_base 在本模块中实际表示“模型镜像内的 Global 字节地址”；
//   3. 第一层的 4 个 256-bit Block 另存于 128 Byte resident cache，帧间无需
//      再访问 Flash 或占用 Local A/B；
//   4. Global、Local A、Local B均使用单端口SRAM。计算只读Active Bank，DMA
//      只写Inactive Bank，因此不需要功耗和面积更大的真双端口Local SRAM。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 上电使用一次QSPI Burst把5084个权重块写入256 KiB Global SRAM。
// - 计算前按Tile把当前/下一层权重复制到8 KiB Local A/B；较大层通过demand miss继续换Tile。
// - 请求地址先判断First Layer常驻区或Local命中，未命中时暂停响应并启动Global到Local复制。
// - Global、Local A和Local B活动信号可用于后续SRAM时钟门控和功耗统计。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：Encoder四级权重存储控制。
// 关键变量/接口：Flash→Global SRAM→Local SRAM A/B→Mesh寄存器；A/B用于当前层计算和下一层预取重叠。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module encoder_weight_hierarchy #(
    // 适配器组织为16组，每组2个512x128宏，同拍拼成256bit。
    parameter GLOBAL_DEPTH_BLOCKS      = 8192,  // 8192 x 32 Byte = 256 KiB
    parameter GLOBAL_INDEX_WIDTH       = 13,

    // 上电从BOOT_FLASH_BASE连续搬运这些块至Global地址0；等于部署镜像块数。
    parameter GLOBAL_MODEL_BLOCKS      = 5084,  // 162688 Byte Encoder packed weights
    parameter BOOT_AUX_BLOCKS          = 0, // 连续镜像尾部，分流至RVQ专用SRAM
    parameter LOCAL_DEPTH_BLOCKS       = 256,  // 256 x 32 Byte = 8 KiB / Bank
    parameter LOCAL_INDEX_WIDTH        = 8,
    parameter FIRST_LAYER_FLASH_BASE   = 24'h000000,
    parameter FIRST_LAYER_BLOCKS       = 4,
    // Flash物理起点与Global SRAM内偏移分离；默认兼容旧工程。
    // Boot将Flash[BOOT_FLASH_BASE + offset]写入Global[offset]。
    parameter BOOT_FLASH_BASE          = FIRST_LAYER_FLASH_BASE,

    // 半周期分频值1使Flash SCLK等于主时钟的一半；50 MHz主时钟对应25 MHz。
    parameter QSPI_HALF_DIV            = 16'd1,
    parameter FLASH_POWER_UP_CYCLES    = 32'd7000,
    parameter QSPI_DUMMY_CYCLES        = 8
) (
    input                                       clk,
    input                                       rst_n,

    // NPU从当前活动Local Bank读取一个256-bit权重块。
    input                                       req_valid,
    output                                      req_ready,
    input              [31:0]                   req_addr,
    output reg                                  rsp_valid,
    input                                       rsp_ready,
    output reg         [255:0]                  rsp_data,

    // Global SRAM -> inactive Local SRAM 的Tile搬运命令。
    input                                       prefetch_valid,
    output                                      prefetch_ready,
    input              [23:0]                   prefetch_global_base,
    input              [15:0]                   prefetch_block_count,
    input                                       prefetch_target_bank,
    output reg                                  prefetch_done,

    // Local Bank切换；activate_first_layer=1时切回常驻第一层。
    input                                       activate_valid,
    output                                      activate_ready,
    input                                       activate_first_layer,
    input                                       activate_bank,
    output reg                                  active_first_layer,
    output reg                                  active_bank,
    output                                      active_bank_valid,
    output reg                                  first_layer_ready,
    output                                      hierarchy_busy,
    output reg                                  hierarchy_error,
    output                                      boot_aux_write,
    output [15:0]                               boot_aux_index,
    output [255:0]                              boot_aux_data,
    // 与 Global SRAM 同一次 Boot 握手：供 FPGA 参数 SRAM 镜像写入。
    output                                      boot_param_write,
    output [12:0]                               boot_param_block,
    output [255:0]                              boot_param_data,

    // 低功耗/调试观察信号：综合时可连接到SRAM宏的CE或sleep控制器。
    output                                      global_sram_active,
    output                                      local_sram_a_active,
    output                                      local_sram_b_active,
    output                                      flash_csn,
    output                                      flash_sclk,
    inout                                       flash_mosi, // P25Q32H SI / IO0
    inout                                       flash_miso, // P25Q32H SO / IO1
    inout                                       flash_wp,   // P25Q32H WPb / IO2
    inout                                       flash_hold  // P25Q32H SIO3
);

localparam [3:0]
C_BOOT_LAUNCH   = 4'd0,
C_BOOT_WAIT     = 4'd1,
C_FIRST_ISSUE   = 4'd2,
C_FIRST_WAIT    = 4'd3,
C_IDLE          = 4'd4,
C_PREFETCH_ISSUE= 4'd5,
C_PREFETCH_WAIT = 4'd6,
C_ERROR         = 4'd7;
reg [3:0] control_state;
// qspi_weight_boot_loader端口为16 bit；独立常量避免integer参数连接产生宽度警告。
localparam [15:0] GLOBAL_MODEL_BLOCKS_16 = GLOBAL_MODEL_BLOCKS + BOOT_AUX_BLOCKS;
reg        dma_cmd_valid;
wire       dma_cmd_ready;
wire       dma_block_valid;
wire       dma_block_ready;
wire [15:0] dma_block_index;
wire [255:0] dma_block_data;
wire       dma_done;
wire       dma_busy;
wire       dma_error;
reg [15:0] copy_index;
reg [GLOBAL_INDEX_WIDTH-1:0] copy_global_base_block;
reg [15:0] copy_block_count;
reg        copy_target_bank;
reg        copy_is_demand;
reg [23:0] bank_base_a;
reg [23:0] bank_base_b;
reg [15:0] bank_blocks_a;
reg [15:0] bank_blocks_b;
reg        bank_valid_a;
reg        bank_valid_b;
reg        prefetch_armed;

// Global SRAM端口：Boot期间由Flash DMA写，其余时间由First/Prefetch DMA读。
wire global_dma_write = (control_state == C_BOOT_WAIT) &&
    dma_block_valid && dma_block_ready && (dma_block_index < GLOBAL_MODEL_BLOCKS);
assign boot_param_write = global_dma_write;
assign boot_param_block = dma_block_index[12:0];
assign boot_param_data = dma_block_data;
assign boot_aux_write = (control_state == C_BOOT_WAIT) && dma_block_valid && dma_block_ready &&
    (dma_block_index >= GLOBAL_MODEL_BLOCKS);
assign boot_aux_index = dma_block_index - GLOBAL_MODEL_BLOCKS;
assign boot_aux_data = dma_block_data;
wire global_copy_read = (control_state == C_FIRST_ISSUE) ||
    (control_state == C_PREFETCH_ISSUE);
wire [GLOBAL_INDEX_WIDTH-1:0] global_addr = global_dma_write ?
    dma_block_index[GLOBAL_INDEX_WIDTH-1:0] :
    (copy_global_base_block + copy_index[GLOBAL_INDEX_WIDTH-1:0]);
wire [255:0] global_rdata;
wire         global_rvalid;
// 连续赋值：组合生成dma_block_ready及其相邻接口信号，表达握手、选择或地址关系。
assign dma_block_ready = (control_state == C_BOOT_WAIT) &&
    (dma_block_index < GLOBAL_MODEL_BLOCKS + BOOT_AUX_BLOCKS);
opentitan_sram_1p_adapter #(
    .WIDTH                       (256),
    .DEPTH                       (GLOBAL_DEPTH_BLOCKS),
    .DATA_BITS_PER_MASK          (8)
) u_global_weight_sram (
    .clk_i                       (clk),
    .rst_ni                      (rst_n),
    .req_i                       (global_dma_write || global_copy_read),
    .write_i                     (global_dma_write),
    .addr_i                      (global_addr),
    .wdata_i                     (dma_block_data),
    .wmask_i                     (32'hffff_ffff),
    .rdata_o                     (global_rdata),
    .rvalid_o                    (global_rvalid)
);

// 第一层常驻区只在Boot完成后写入，正常计算期间只读。
wire first_copy_write = (control_state == C_FIRST_WAIT) && global_rvalid;
reg  read_pending;
reg  read_source_first_q;
reg  read_bank_q;
wire selected_bank_valid = active_first_layer ? first_layer_ready :
    (active_bank ? bank_valid_b : bank_valid_a);
wire [23:0] selected_bank_base = active_first_layer ?
    FIRST_LAYER_FLASH_BASE :
    (active_bank ? bank_base_b : bank_base_a);
wire [15:0] selected_bank_blocks = active_first_layer ?
    FIRST_LAYER_BLOCKS :
    (active_bank ? bank_blocks_b : bank_blocks_a);
wire [31:0] request_offset = req_addr - {8'd0, selected_bank_base};
wire [26:0] request_block_index = request_offset[31:5];
wire request_in_range = (req_addr[31:24] == 8'd0) &&
    (req_addr >= {8'd0, selected_bank_base}) &&
    (request_offset[4:0] == 5'd0) &&
    (request_block_index < selected_bank_blocks);
wire npu_read_fire = req_valid && req_ready && request_in_range;

// Local容量只需覆盖一个8-output-channel Tile。若下一output_group不在当前
// Local Bank中，req_ready保持0，本模块自动把对应8 KiB窗口搬到另一Bank。
wire [GLOBAL_INDEX_WIDTH-1:0] request_global_block =
    req_addr[GLOBAL_INDEX_WIDTH+4:5];
wire [GLOBAL_INDEX_WIDTH-1:0] demand_tile_base_block = {
request_global_block[GLOBAL_INDEX_WIDTH-1:LOCAL_INDEX_WIDTH],
{LOCAL_INDEX_WIDTH{1'b0}}
};
wire [GLOBAL_INDEX_WIDTH:0] demand_blocks_remaining =
    GLOBAL_MODEL_BLOCKS - demand_tile_base_block;
wire demand_address_valid = (req_addr[31:24] == 8'd0) &&
    (req_addr[4:0] == 5'd0) &&
    (request_global_block < GLOBAL_MODEL_BLOCKS);
wire first_sram_read = npu_read_fire && active_first_layer;
wire bank_a_sram_read = npu_read_fire && !active_first_layer && !active_bank;
wire bank_b_sram_read = npu_read_fire && !active_first_layer && active_bank;
wire local_copy_write = (control_state == C_PREFETCH_WAIT) && global_rvalid;
wire bank_a_sram_write = local_copy_write && !copy_target_bank;
wire bank_b_sram_write = local_copy_write &&  copy_target_bank;
wire [$clog2(FIRST_LAYER_BLOCKS)-1:0] first_sram_addr = first_copy_write ?
    copy_index[$clog2(FIRST_LAYER_BLOCKS)-1:0] :
    request_block_index[$clog2(FIRST_LAYER_BLOCKS)-1:0];
wire [LOCAL_INDEX_WIDTH-1:0] bank_a_sram_addr = bank_a_sram_write ?
    copy_index[LOCAL_INDEX_WIDTH-1:0] :
    request_block_index[LOCAL_INDEX_WIDTH-1:0];
wire [LOCAL_INDEX_WIDTH-1:0] bank_b_sram_addr = bank_b_sram_write ?
    copy_index[LOCAL_INDEX_WIDTH-1:0] :
    request_block_index[LOCAL_INDEX_WIDTH-1:0];
wire [255:0] first_sram_rdata;
wire [255:0] bank_a_sram_rdata;
wire [255:0] bank_b_sram_rdata;
wire first_sram_rvalid;
wire bank_a_sram_rvalid;
wire bank_b_sram_rvalid;
opentitan_sram_1p_adapter #(
    .WIDTH                       (256),
    .DEPTH                       (FIRST_LAYER_BLOCKS),
    .DATA_BITS_PER_MASK          (8)
) u_first_layer_local (
    .clk_i                       (clk),
    .rst_ni                      (rst_n),
    .req_i                       (first_copy_write || first_sram_read),
    .write_i                     (first_copy_write),
    .addr_i                      (first_sram_addr),
    .wdata_i                     (global_rdata),
    .wmask_i                     (32'hffff_ffff),
    .rdata_o                     (first_sram_rdata),
    .rvalid_o                    (first_sram_rvalid)
);

opentitan_sram_1p_adapter #(
    .WIDTH                       (256),
    .DEPTH                       (LOCAL_DEPTH_BLOCKS),
    .DATA_BITS_PER_MASK          (8)
) u_local_weight_sram_a (
    .clk_i                       (clk),
    .rst_ni                      (rst_n),
    .req_i                       (bank_a_sram_write || bank_a_sram_read),
    .write_i                     (bank_a_sram_write),
    .addr_i                      (bank_a_sram_addr),
    .wdata_i                     (global_rdata),
    .wmask_i                     (32'hffff_ffff),
    .rdata_o                     (bank_a_sram_rdata),
    .rvalid_o                    (bank_a_sram_rvalid)
);

opentitan_sram_1p_adapter #(
    .WIDTH                       (256),
    .DEPTH                       (LOCAL_DEPTH_BLOCKS),
    .DATA_BITS_PER_MASK          (8)
) u_local_weight_sram_b (
    .clk_i                       (clk),
    .rst_ni                      (rst_n),
    .req_i                       (bank_b_sram_write || bank_b_sram_read),
    .write_i                     (bank_b_sram_write),
    .addr_i                      (bank_b_sram_addr),
    .wdata_i                     (global_rdata),
    .wmask_i                     (32'hffff_ffff),
    .rdata_o                     (bank_b_sram_rdata),
    .rvalid_o                    (bank_b_sram_rvalid)
);

wire selected_read_rvalid = read_source_first_q ? first_sram_rvalid :
    (read_bank_q ? bank_b_sram_rvalid :
    bank_a_sram_rvalid);
wire [255:0] selected_read_data = read_source_first_q ? first_sram_rdata :
    (read_bank_q ? bank_b_sram_rdata :
    bank_a_sram_rdata);
wire [40:0] prefetch_end_address =
    {17'd0, prefetch_global_base} + ({25'd0, prefetch_block_count} << 5);
wire [40:0] model_end_address = ({25'd0, GLOBAL_MODEL_BLOCKS} << 5);
wire prefetch_command_valid = (prefetch_block_count != 0) &&
    (prefetch_global_base[4:0] == 5'd0) &&
    (prefetch_end_address <= model_end_address);
wire [15:0] prefetch_local_blocks =
    (prefetch_block_count > LOCAL_DEPTH_BLOCKS) ?
    LOCAL_DEPTH_BLOCKS : prefetch_block_count;
// 连续赋值：组合生成req_ready及其相邻接口信号，表达握手、选择或地址关系。
assign req_ready = selected_bank_valid && request_in_range &&
    !rsp_valid && !read_pending &&
    ((control_state == C_IDLE) ||
    ((control_state == C_PREFETCH_ISSUE ||
    control_state == C_PREFETCH_WAIT) &&
    (active_first_layer ||
    (active_bank != copy_target_bank))));
// 连续赋值：组合生成prefetch_ready及其相邻接口信号，表达握手、选择或地址关系。
assign prefetch_ready = prefetch_armed && (control_state == C_IDLE) &&
    !rsp_valid && !read_pending;
// 连续赋值：组合生成activate_ready及其相邻接口信号，表达握手、选择或地址关系。
assign activate_ready = (control_state == C_IDLE) && !rsp_valid &&
    !read_pending &&
    (activate_first_layer ? first_layer_ready :
    (activate_bank ? bank_valid_b : bank_valid_a));
// 连续赋值：组合生成active_bank_valid及其相邻接口信号，表达握手、选择或地址关系。
assign active_bank_valid = selected_bank_valid;
assign hierarchy_busy = (control_state != C_IDLE) || dma_busy || dma_cmd_valid;
// 连续赋值：组合生成global_sram_active及其相邻接口信号，表达握手、选择或地址关系。
assign global_sram_active = global_dma_write || global_copy_read;
assign local_sram_a_active = bank_a_sram_write || bank_a_sram_read;
// 连续赋值：组合生成local_sram_b_active及其相邻接口信号，表达握手、选择或地址关系。
assign local_sram_b_active = bank_b_sram_write || bank_b_sram_read;
qspi_weight_boot_loader #(
    .QSPI_HALF_DIV               (QSPI_HALF_DIV),
    .POWER_UP_CYCLES             (FLASH_POWER_UP_CYCLES),
    .QSPI_DUMMY_CYCLES           (QSPI_DUMMY_CYCLES)
) u_flash_boot_dma (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .cmd_valid                   (dma_cmd_valid),
    .cmd_ready                   (dma_cmd_ready),
    .cmd_flash_base              (BOOT_FLASH_BASE),
    .cmd_block_count             (GLOBAL_MODEL_BLOCKS_16),
    .block_valid                 (dma_block_valid),
    .block_ready                 (dma_block_ready),
    .block_index                 (dma_block_index),
    .block_data                  (dma_block_data),
    .done                        (dma_done),
    .busy                        (dma_busy),
    .error                       (dma_error),
    .flash_csn                   (flash_csn),
    .flash_sclk                  (flash_sclk),
    .flash_dq                    ({flash_hold, flash_wp, flash_miso, flash_mosi})
);

// 时序逻辑：在时钟沿更新control_state、dma_cmd_valid、copy_index、copy_global_base_block、copy_block_count、copy_target_bank；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        control_state        <= C_BOOT_LAUNCH;
        dma_cmd_valid        <= 1'b0;
        copy_index           <= 16'd0;
        copy_global_base_block <= {GLOBAL_INDEX_WIDTH{1'b0}};
        copy_block_count     <= 16'd0;
        copy_target_bank     <= 1'b0;
        copy_is_demand       <= 1'b0;
        bank_base_a          <= 24'd0;
        bank_base_b          <= 24'd0;
        bank_blocks_a        <= 16'd0;
        bank_blocks_b        <= 16'd0;
        bank_valid_a         <= 1'b0;
        bank_valid_b         <= 1'b0;
        prefetch_armed       <= 1'b1;
        active_first_layer   <= 1'b0;
        active_bank          <= 1'b0;
        first_layer_ready    <= 1'b0;
        hierarchy_error      <= 1'b0;
        prefetch_done        <= 1'b0;
        read_pending         <= 1'b0;
        read_source_first_q  <= 1'b0;
        read_bank_q          <= 1'b0;
        rsp_valid            <= 1'b0;
        rsp_data             <= 256'd0;
    end else begin
        prefetch_done <= 1'b0;
        if (!prefetch_valid)
            prefetch_armed <= 1'b1;
        if (dma_error) begin
            hierarchy_error <= 1'b1;
            control_state   <= C_ERROR;
        end

        // NPU读响应必须在rsp_ready反压期间保持不变。
        if (npu_read_fire) begin
            read_pending        <= 1'b1;
            read_source_first_q <= active_first_layer;
            read_bank_q         <= active_bank;
        end
        if (selected_read_rvalid && read_pending) begin
            rsp_valid    <= 1'b1;
            rsp_data     <= selected_read_data;
            read_pending <= 1'b0;
        end
        if (rsp_valid && rsp_ready)
            rsp_valid <= 1'b0;
        if (activate_valid && activate_ready) begin
            active_first_layer <= activate_first_layer;
            if (!activate_first_layer)
                active_bank <= activate_bank;
        end
        case (control_state)

            // 上电搬运：发起 Flash -> Global SRAM 的整段加载请求。
            C_BOOT_LAUNCH: begin

                // 防止部署镜像超过Global SRAM后因地址截断而静默覆盖旧权重。
                if (GLOBAL_MODEL_BLOCKS > GLOBAL_DEPTH_BLOCKS ||
                    GLOBAL_MODEL_BLOCKS + BOOT_AUX_BLOCKS > 65535) begin
                    dma_cmd_valid   <= 1'b0;
                    hierarchy_error <= 1'b1;
                    control_state   <= C_ERROR;
                end else begin
                    dma_cmd_valid <= 1'b1;
                    if (dma_cmd_valid && dma_cmd_ready) begin
                        dma_cmd_valid <= 1'b0;
                        control_state <= C_BOOT_WAIT;
                    end
                end
            end

            // 等待 DMA 实际结束；不能把“请求已接收”当成“权重已可读”。
            C_BOOT_WAIT: begin
                if (dma_done) begin
                    copy_index             <= 16'd0;
                    copy_global_base_block <=
                        FIRST_LAYER_FLASH_BASE[23:5];
                    copy_block_count       <= FIRST_LAYER_BLOCKS;
                    control_state          <= C_FIRST_ISSUE;
                end
            end

            // SRAM 同步读分为请求与返回两步，避免同拍使用尚未返回的数据。
            C_FIRST_ISSUE: begin
                control_state <= C_FIRST_WAIT;
            end
            C_FIRST_WAIT: begin
                if (global_rvalid) begin
                    if ((copy_index + 1'b1) >= copy_block_count) begin
                        first_layer_ready  <= 1'b1;
                        active_first_layer<= 1'b1;
                        active_bank       <= 1'b0;
                        control_state     <= C_IDLE;
                    end else begin
                        copy_index    <= copy_index + 1'b1;
                        control_state <= C_FIRST_ISSUE;
                    end
                end
            end

            // 空闲仲裁：处理层准备请求，或响应计算端的 Local Tile 未命中。
            C_IDLE: begin
                if (prefetch_valid && prefetch_ready) begin
                    prefetch_armed <= 1'b0;
                    if (!prefetch_command_valid) begin
                        hierarchy_error <= 1'b1;
                        prefetch_done   <= 1'b1;
                    end else begin
                        copy_index <= 16'd0;
                        copy_global_base_block <=
                            prefetch_global_base[23:5];
                        copy_block_count <= prefetch_local_blocks;
                        copy_target_bank <= prefetch_target_bank;
                        copy_is_demand   <= 1'b0;
                        control_state    <= C_PREFETCH_ISSUE;
                    end
                end else if (req_valid && selected_bank_valid &&
                    !request_in_range) begin
                    if (!demand_address_valid) begin
                        hierarchy_error <= 1'b1;
                        control_state   <= C_ERROR;
                    end else begin
                        copy_index             <= 16'd0;
                        copy_global_base_block <= demand_tile_base_block;
                        copy_block_count <=
                            (demand_blocks_remaining > LOCAL_DEPTH_BLOCKS) ?
                            LOCAL_DEPTH_BLOCKS : demand_blocks_remaining;
                        copy_target_bank <= active_first_layer ?
                            1'b0 : ~active_bank;
                        copy_is_demand   <= 1'b1;
                        control_state    <= C_PREFETCH_ISSUE;
                    end
                end
            end

            // 从 Global 发起一块 256 bit 读取，随后在 WAIT 状态写入 Local。
            C_PREFETCH_ISSUE: begin
                control_state <= C_PREFETCH_WAIT;
            end
            C_PREFETCH_WAIT: begin
                if (global_rvalid) begin
                    if ((copy_index + 1'b1) >= copy_block_count) begin
                        if (copy_target_bank) begin
                            bank_base_b   <=
                                {copy_global_base_block, 5'd0};
                            bank_blocks_b <= copy_block_count;
                            bank_valid_b  <= 1'b1;
                        end else begin
                            bank_base_a   <=
                                {copy_global_base_block, 5'd0};
                            bank_blocks_a <= copy_block_count;
                            bank_valid_a  <= 1'b1;
                        end
                        if (copy_is_demand) begin
                            active_first_layer <= 1'b0;
                            active_bank        <= copy_target_bank;
                        end else begin
                            prefetch_done <= 1'b1;
                        end
                        control_state <= C_IDLE;
                    end else begin
                        copy_index    <= copy_index + 1'b1;
                        control_state <= C_PREFETCH_ISSUE;
                    end
                end
            end
            C_ERROR: begin
                control_state <= C_ERROR;
            end
            default: begin
                hierarchy_error <= 1'b1;
                control_state   <= C_ERROR;
            end
        endcase
    end
end

endmodule
