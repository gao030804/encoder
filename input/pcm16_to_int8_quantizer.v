// ============================================================================
// 中文阅读导引（当前实现）
// PCM前端量化：将signed 16-bit采样按配置缩放、舍入并饱和到signed INT8。
// 缩放参数决定真实PCM幅度与整数激活的对应关系，需要与训练/整数参考模型一致。
// ============================================================================
//=============================================================================
// 模块：pcm16_to_int8_quantizer
// 功能：把 signed PCM16 样本重新量化为 signed INT8 激活值。
//
// 公式：q = sat_int8(round(pcm * multiplier / 2^shift) + zero_point)
// multiplier、shift 和 zero_point 必须来自模型导出或离线校准。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 定点公式为q=sat8(round_half_away_from_zero(pcm*multiplier/2^shift)+zero_point)。
// - 乘法与舍入使用64-bit signed中间值，避免PCM16乘INT32时提前溢出。
// - 输出范围为signed INT8 [-128,127]，量化参数必须与训练导出参数一致。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：PCM16到INT8定点量化器。
// 关键变量/接口：执行乘法、对称舍入、右移、Zero-point和饱和；64-bit中间值防止乘法截断。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
// 为兼容已有例化保留旧模块名；当前输出数据已经升级为signed INT16。
module pcm16_to_int8_quantizer (
    input                                       clk,
    input                                       rst_n,
    input                                       in_valid,
    output                                      in_ready,
    input       signed [15:0]                   in_pcm,
    input                                       in_last,
    input       signed [31:0]                   cfg_multiplier,
    input              [5:0]                    cfg_shift,
    input       signed [7:0]                    cfg_zero_point,
    output reg                                  out_valid,
    input                                       out_ready,
    output reg  signed [15:0]                   out_data,
    output reg                                  out_last
);

reg signed [63:0] pcm_ext;
reg signed [63:0] mult_ext;
reg signed [63:0] scaled;
reg signed [63:0] magnitude;
reg signed [63:0] rounded;
reg signed [63:0] shifted;
reg signed [63:0] requantized;
reg signed [15:0] quantized;
// 连续赋值：组合生成in_ready及其相邻接口信号，表达握手、选择或地址关系。
assign in_ready = !out_valid || out_ready;

// 组合逻辑：根据当前输入计算pcm_ext、mult_ext、scaled、magnitude、rounded、shifted；本逻辑块不保存跨周期状态。
always @(*) begin
    pcm_ext     = {{48{in_pcm[15]}}, in_pcm};
    mult_ext    = {{32{cfg_multiplier[31]}}, cfg_multiplier};
    scaled      = $signed(pcm_ext) * $signed(mult_ext);
    magnitude   = 64'sd0;
    rounded     = 64'sd0;
    shifted     = 64'sd0;
    requantized = 64'sd0;
    quantized   = 16'sd0;

    // 对称舍入。负数按绝对值舍入后恢复符号，避免算术右移偏向负无穷。
    if (cfg_shift == 0) begin
        shifted = scaled;
    end else if (scaled >= 0) begin
        rounded = scaled + (64'sd1 << (cfg_shift - 1'b1));
        shifted = rounded >>> cfg_shift;
    end else begin
        magnitude = -scaled;
        rounded   = magnitude + (64'sd1 << (cfg_shift - 1'b1));
        shifted   = -(rounded >>> cfg_shift);
    end
    requantized = shifted + {{56{cfg_zero_point[7]}}, cfg_zero_point};
    if (requantized > 64'sd32767) begin
        quantized = 16'sd32767;
    end else if (requantized < -64'sd32768) begin
        quantized = -16'sd32768;
    end else begin
        quantized = requantized[15:0];
    end
end

// 时序逻辑：在时钟沿更新out_valid、out_data、out_last；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        out_valid <= 1'b0;
        out_data  <= 16'sd0;
        out_last  <= 1'b0;
    end else if (in_ready) begin
        out_valid <= in_valid;
        if (in_valid) begin
            out_data <= quantized;
            out_last <= in_last;
        end
    end
end

endmodule
