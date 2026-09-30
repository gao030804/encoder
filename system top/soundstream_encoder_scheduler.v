// ============================================================================
// 中文阅读导引（当前实现）
// 帧级状态机：等待首层权重就绪 → 准备本层 → 启动引擎 → 等待完成 → 搬运下一层权重 → 切换Bank。
// 残差入口先执行capture，残差出口等待Add写回完成。done为帧完成脉冲，不能代替单层engine_done。
// 当前层表包含63个物理Conv1d，支持Dense、低秩Pointwise和Depthwise调度。
// ============================================================================
//=============================================================================
// 模块：soundstream_encoder_scheduler
// 功能：63个物理Conv1d层的自动调度、A/B权重预取、Residual capture/add控制。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 调度循环覆盖physical_layer=0..62。每层依次执行Residual Capture、参数准备、engine_start、等待计算完成。
// - 下一层权重可与当前层计算并行预取；只有prefetch_done后才能切换Local SRAM A/B。
// - 所有状态只在对应valid&&ready或done事件发生时推进，不能依赖固定Flash/SRAM延迟。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：63层Encoder层调度器。
// 关键变量/接口：依次完成残差捕获、参数准备、权重Bank激活、卷积启动和完成等待，并预取下一层权重。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module soundstream_encoder_scheduler #(
    parameter LAST_LAYER = 6'd62
) (
    input                                       clk,
    input                                       rst_n,
    input                                       start,
    output                                      start_ready,
    output                                      busy,
    output reg                                  done,
    output reg                                  error,
    input                                       first_layer_weight_ready,
    input                                       active_weight_is_first_layer,
    input                                       weight_error,
    output                                      weight_prefetch_valid,
    input                                       weight_prefetch_ready,
    output             [23:0]                   weight_prefetch_flash_base,
    output             [15:0]                   weight_prefetch_block_count,
    output                                      weight_prefetch_target_bank,
    input                                       weight_prefetch_done,
    output                                      weight_activate_valid,
    input                                       weight_activate_ready,
    output                                      weight_activate_first_layer,
    output                                      weight_activate_bank,
    output                                      engine_start,
    input                                       engine_done,
    input                                       engine_error,
    output                                      capture_start,
    input                                       capture_start_ready,
    input                                       capture_done,
    input                                       capture_error,
    output                                      layer_prepare_valid,
    input                                       layer_prepare_ready,
    output reg         [5:0]                    physical_layer,
    output             [5:0]                    logical_layer,
    output reg                                  current_buffer_select,
    output                                      residual_add_enable,
    output             [8:0]                    cfg_cin,
    output             [8:0]                    cfg_cout,
    output             [4:0]                    cfg_kernel,
    output             [3:0]                    cfg_stride,
    output             [3:0]                    cfg_dilation,
    output             [7:0]                    cfg_left_pad,
    output             [8:0]                    cfg_input_length,
    output             [8:0]                    cfg_output_length,
    output             [11:0]                   cfg_k_total,
    output             [9:0]                    cfg_k_groups,
    output             [5:0]                    cfg_n_groups,
    output                                      cfg_is_depthwise,
    output                                      cfg_relu_enable,
    output                                      cfg_elu_enable,
    output             [23:0]                   cfg_weight_flash_base,
    output             [15:0]                   cfg_weight_block_count
);

localparam [3:0] S_IDLE=0, S_CAPTURE_START=1, S_CAPTURE_WAIT=2,
S_PREPARE=3, S_ENGINE_START=4, S_RUN=5,
S_WAIT_PREFETCH=6, S_ACTIVATE_NEXT=7,
S_RETURN_FIRST=8, S_DONE=9, S_ERROR=10;
reg [3:0] state;
reg prefetch_accepted;
reg prefetch_done_seen;
wire rom_valid;
wire rom_residual_capture;
wire rom_residual_add;
soundstream_encoder_layer_rom u_current_rom (
    .layer_index                 (physical_layer),
    .valid                       (rom_valid),
    .logical_layer               (logical_layer),
    .cin                         (cfg_cin),
    .cout                        (cfg_cout),
    .kernel                      (cfg_kernel),
    .stride                      (cfg_stride),
    .dilation                    (cfg_dilation),
    .left_pad                    (cfg_left_pad),
    .input_length                (cfg_input_length),
    .output_length               (cfg_output_length),
    .k_total                     (cfg_k_total),
    .k_groups                    (cfg_k_groups),
    .n_groups                    (cfg_n_groups),
    .is_depthwise                (cfg_is_depthwise),
    .relu_enable                 (cfg_relu_enable),
    .elu_enable                  (cfg_elu_enable),
    .residual_capture            (rom_residual_capture),
    .residual_add                (rom_residual_add),
    .weight_flash_base           (cfg_weight_flash_base),
    .weight_block_count          (cfg_weight_block_count)
);

wire [5:0] next_layer = physical_layer + 1'b1;
wire next_valid;
wire [5:0] unused_logical;
wire [8:0] unused_cin,unused_cout,unused_lin,unused_lout;
wire [4:0] unused_kernel;
wire [3:0] unused_stride,unused_dilation;
wire [7:0] unused_pad;
wire [11:0] unused_ktotal;
wire [9:0] unused_kgroups;
wire [5:0] unused_ngroups;
wire unused_dw,unused_relu,unused_elu,unused_cap,unused_add;
wire [23:0] next_weight_base;
wire [15:0] next_weight_blocks;
soundstream_encoder_layer_rom u_next_rom (
    .layer_index                 (next_layer),
    .valid                       (next_valid),
    .logical_layer               (unused_logical),
    .cin                         (unused_cin),
    .cout                        (unused_cout),
    .kernel                      (unused_kernel),
    .stride                      (unused_stride),
    .dilation                    (unused_dilation),
    .left_pad                    (unused_pad),
    .input_length                (unused_lin),
    .output_length               (unused_lout),
    .k_total                     (unused_ktotal),
    .k_groups                    (unused_kgroups),
    .n_groups                    (unused_ngroups),
    .is_depthwise                (unused_dw),
    .relu_enable                 (unused_relu),
    .elu_enable                  (unused_elu),
    .residual_capture            (unused_cap),
    .residual_add                (unused_add),
    .weight_flash_base           (next_weight_base),
    .weight_block_count          (next_weight_blocks)
);

// 连续赋值：组合生成busy及其相邻接口信号，表达握手、选择或地址关系。
assign busy = (state != S_IDLE);
assign start_ready = (state == S_IDLE) && first_layer_weight_ready &&
    active_weight_is_first_layer;
// 连续赋值：组合生成capture_start及其相邻接口信号，表达握手、选择或地址关系。
assign capture_start = (state == S_CAPTURE_START);
assign layer_prepare_valid = (state == S_PREPARE);
// 连续赋值：组合生成engine_start及其相邻接口信号，表达握手、选择或地址关系。
assign engine_start = (state == S_ENGINE_START);
assign residual_add_enable = rom_residual_add;

// 8 KiB Local Bank只保存一个输出通道Tile。当前层运行时，Hierarchy可能用
// 另一Bank处理Local miss，因此下一层首Tile在当前层完成后再装入，避免互相覆盖。
wire prefetch_phase = (state == S_WAIT_PREFETCH);
// 连续赋值：组合生成weight_prefetch_valid及其相邻接口信号，表达握手、选择或地址关系。
assign weight_prefetch_valid = prefetch_phase &&
    (physical_layer < LAST_LAYER) &&
    !prefetch_accepted;
// 连续赋值：组合生成weight_prefetch_flash_base及其相邻接口信号，表达握手、选择或地址关系。
assign weight_prefetch_flash_base = next_weight_base;
assign weight_prefetch_block_count = next_weight_blocks;

// physical 1进B，physical 2进A，之后交替。
// 连续赋值：组合生成weight_prefetch_target_bank及其相邻接口信号，表达握手、选择或地址关系。
assign weight_prefetch_target_bank = next_layer[0];
assign weight_activate_valid = (state == S_ACTIVATE_NEXT) ||
    (state == S_RETURN_FIRST);
// 连续赋值：组合生成weight_activate_first_layer及其相邻接口信号，表达握手、选择或地址关系。
assign weight_activate_first_layer = (state == S_RETURN_FIRST);
assign weight_activate_bank = next_layer[0];

// 时序逻辑：在时钟沿更新state、physical_layer、current_buffer_select、prefetch_accepted、prefetch_done_seen、done；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state                 <= S_IDLE;
        physical_layer        <= 6'd0;
        current_buffer_select <= 1'b0;
        prefetch_accepted     <= 1'b0;
        prefetch_done_seen    <= 1'b0;
        done                  <= 1'b0;
        error                 <= 1'b0;
    end else begin
        done <= 1'b0;
        if (weight_prefetch_valid && weight_prefetch_ready)
            prefetch_accepted <= 1'b1;
        if (weight_prefetch_done)
            prefetch_done_seen <= 1'b1;
        if (weight_error || engine_error || capture_error) begin
            error <= 1'b1;
            state <= S_ERROR;
        end else begin
            case (state)
                S_IDLE: begin
                    if (start && start_ready) begin
                        physical_layer     <= 6'd0;
                        prefetch_accepted  <= 1'b0;
                        prefetch_done_seen <= 1'b0;
                        state <= S_PREPARE;
                    end
                end
                S_CAPTURE_START:
                    if (capture_start && capture_start_ready)
                    state <= S_CAPTURE_WAIT;
                S_CAPTURE_WAIT:
                    if (capture_done)
                    state <= S_PREPARE;

                // 配置交接完成后，下一拍才发出单层 engine_start。
                S_PREPARE:
                    if (layer_prepare_valid && layer_prepare_ready)
                    state <= S_ENGINE_START;
                S_ENGINE_START: begin

                    // 为本层结束后的权重准备清除记录；末层没有下一层，直接标记完成。
                    // 当前版本在 S_WAIT_PREFETCH 发起请求，不与本层计算重叠。
                    prefetch_accepted  <= (physical_layer == LAST_LAYER);
                    prefetch_done_seen <= (physical_layer == LAST_LAYER);
                    state <= S_RUN;
                end
                S_RUN: begin
                    if (engine_done) begin
                        current_buffer_select <= ~current_buffer_select;
                        if (physical_layer == LAST_LAYER)
                            state <= S_RETURN_FIRST;
                        else
                        state <= S_WAIT_PREFETCH;
                    end
                end
                S_WAIT_PREFETCH:
                    if (prefetch_done_seen || weight_prefetch_done)
                    state <= S_ACTIVATE_NEXT;

                // Bank 切换握手成功，才能递增层号并准备下一层。
                S_ACTIVATE_NEXT: begin
                    if (weight_activate_valid && weight_activate_ready) begin
                        physical_layer     <= physical_layer + 1'b1;
                        prefetch_accepted  <= 1'b0;
                        prefetch_done_seen <= 1'b0;
                        state <= next_valid && unused_cap ? S_CAPTURE_START : S_PREPARE;
                    end
                end
                S_RETURN_FIRST:
                    if (weight_activate_valid && weight_activate_ready)
                    state <= S_DONE;
                S_DONE: begin
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                // 错误保持到复位；不能把异常当作一帧正常完成。
                S_ERROR:
                    state <= S_ERROR;
                default: begin
                    error <= 1'b1;
                    state <= S_ERROR;
                end
            endcase
        end
    end
end

endmodule
