// ============================================================================
// 中文阅读导引（当前实现）
// 逐输出通道使用独立Bias、Multiplier、Shift和Zero Point。
// 运算顺序：INT32累加值加Bias并饱和 → INT64乘法 → 舍入/右移 → 加Zero Point → INT8饱和。
// in_ready=!post_valid||post_ready允许旧输出取走时同拍接收新输入。
// 当前算术主要在组合逻辑内完成，输出寄存器可连续接收，但不等同于乘法器内部多级流水。
// ============================================================================
//=============================================================================
// 模块：postprocess_8lane
// 功能：对 8 路 INT32 卷积累加值进行逐输出通道重新量化。
//
// 每一路的数据路径：
//   INT32 accumulator + INT32 bias（带饱和）
//   -> INT32 × INT32 = INT64
//   -> 对称舍入并算术右移
//   -> 加 signed INT8 zero_point
//   -> 显式饱和到 signed INT8
//
// valid/ready 规则：post_valid=1 且 post_ready=0 时，输出保持不变。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 每个输出lane独立使用bias、multiplier、shift和zero_point，支持per-Cout量化参数。
// - 计算链为INT32加Bias、INT64乘法、half-away-from-zero舍入、右移和INT8饱和。
// - post_valid被反压时post_data保持不变；不能在ready=0时覆盖正在等待的数据。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：八路逐输出通道后处理。
// 关键变量/接口：每lane独立执行Bias、Multiplier、Shift、Zero-point、舍入及INT8饱和。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
// W8A16合同：INT40累加与INT40 Bias相加，乘INT32系数后舍入并饱和为INT16。
// 部署文件中的Bias使用INT64容器，本模块读取每个容器的低40位二补码。
module postprocess_8lane (
    input                                       clk,
    input                                       rst_n,
    input                                       in_valid,
    output                                      in_ready,
    input              [319:0]                  acc_data,
    input              [511:0]                  bias_data,
    input              [255:0]                  mult_data,
    input              [63:0]                   shift_data,
    input              [63:0]                   zero_point_data,
    output reg                                  post_valid,
    input                                       post_ready,
    output reg         [127:0]                  post_data
);

integer n;
reg signed [39:0] av;
reg signed [39:0] bv;
reg signed [31:0] mv;
reg signed [15:0] zv;
reg        [7:0]  sv;
reg signed [40:0] bias_sum;
reg signed [39:0] biased_int40;
reg signed [79:0] scaled, rounded, magnitude, shifted, requantized;
reg signed [15:0] sat;
reg        [127:0] calc;
// 连续赋值：组合生成in_ready及其相邻接口信号，表达握手、选择或地址关系。
assign in_ready = !post_valid || post_ready;

// 组合逻辑：根据当前输入计算calc、av、bv、mv、zv、sv；本逻辑块不保存跨周期状态。
always @(*) begin
    calc        = 128'd0;
    av          = 40'sd0;
    bv          = 40'sd0;
    mv          = 32'sd0;
    zv          = 16'sd0;
    sv          = 8'd0;
    bias_sum    = 41'sd0;
    biased_int40 = 40'sd0;
    scaled      = 80'sd0;
    rounded     = 80'sd0;
    magnitude   = 80'sd0;
    shifted     = 80'sd0;
    requantized = 80'sd0;
    sat         = 16'sd0;
    for (n = 0; n < 8; n = n + 1) begin

        // lane n 对应一个输出通道，四种量化参数均按 lane 独立配置。
        av = acc_data[40*n +: 40];
        // 导出文件每个Bias占64 bit，低40 bit为有效二补码。
        bv = bias_data[64*n +: 40];
        mv = mult_data[32*n +: 32];
        sv = shift_data[8*n +: 8];
        zv = {{8{zero_point_data[8*n+7]}},zero_point_data[8*n +: 8]};

        // 在 64 bit 中先做加法，再饱和回 INT32，避免 acc+bias 回绕。
        bias_sum = {av[39],av} + {bv[39],bv};

        // Golden模型约定：累加值与Bias相加后仍属于INT40域。
        // 因此先用41位保存精确加法结果，再饱和到INT40，最后参与定点乘法。
        // 这一步避免正溢出/负溢出时发生二补码回绕，并与Python整数参考模型一致。
        if (bias_sum > 41'sd549755813887)
            biased_int40 = 40'sh7f_ffff_ffff;
        else if (bias_sum < -41'sd549755813888)
            biased_int40 = 40'sh80_0000_0000;
        else
            biased_int40 = bias_sum[39:0];

        scaled = $signed(biased_int40) * $signed(mv);

        // 对负数也从绝对值方向舍入。
        // 例如 -4/2 必须仍为 -2，旧式 (scaled-half)>>>shift 会得到 -3。
        if (sv == 0) begin
            shifted = scaled;
        end else if (scaled >= 0) begin
            rounded = scaled + (80'sd1 << (sv - 1'b1));
            shifted = rounded >>> sv;
        end else begin
            magnitude = -scaled;
            rounded   = magnitude + (80'sd1 << (sv - 1'b1));
            shifted   = -(rounded >>> sv);
        end

        // 对称量化时 zero_point 配置为 0；非对称量化可逐通道配置。
        requantized = shifted + {{64{zv[15]}}, zv};
        if (requantized > 80'sd32767) begin
            sat = 16'sd32767;
        end else if (requantized < -80'sd32768) begin
            sat = -16'sd32768;
        end else begin
            sat = requantized[15:0];
        end
        calc[16*n +: 16] = sat;
    end
end

// 时序逻辑：在时钟沿更新post_valid、post_data；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        post_valid <= 1'b0;
        post_data  <= 128'd0;
    end else if (in_ready) begin
        post_valid <= in_valid;
        if (in_valid) begin
            post_data <= calc;
        end
    end
end

endmodule
