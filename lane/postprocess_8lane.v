`timescale 1ns/1ps
// ============================================================================
// W8A16逐Cout后处理：三级Elastic Pipeline，稳态II=1（每拍一组8路）。
// PP0：INT40 Acc + INT40 Bias -> 41位精确和 -> 饱和回INT40。
// PP1：INT40 x signed INT32 -> 完整signed INT72乘积。
// PP2：half-away-from-zero舍入/右移 -> signed INT8 Zero Point -> INT16饱和。
// Bias外部容器为64位/路，只有低40位有效；不是新增INT64 Bias算法。
// 每级独立握手，数据、逐通道参数与Tag作为同一事务一起保持/推进。
// 输入在N沿接收，PP2在N+2沿更新，N+3沿可被下游接收；比原单级多2拍。
// ============================================================================
module postprocess_8lane #(
    parameter TAG_WIDTH = 1
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 in_valid,
    output wire                 in_ready,
    input  wire [319:0]         acc_data,
    input  wire [511:0]         bias_data,
    input  wire [255:0]         mult_data,
    input  wire [63:0]          shift_data,
    input  wire [63:0]          zero_point_data,
    output reg                  post_valid,
    input  wire                 post_ready,
    output reg  [127:0]         post_data,
    // Tag不参与算术。Dense使用输出时间位置m；单笔Depthwise可固定为0。
    input  wire [TAG_WIDTH-1:0] in_tag,
    output reg  [TAG_WIDTH-1:0] post_tag
);
    reg pp0_valid, pp1_valid;
    reg [319:0] pp0_biased;     // 8 x INT40，Bias饱和后的值
    reg [255:0] pp0_mult;       // 与PP0数据同笔的8个INT32 multiplier
    reg [63:0] pp0_shift, pp0_zp;
    reg [575:0] pp1_product;    // 8 x INT72，精确乘积，不截断有效位
    reg [63:0] pp1_shift, pp1_zp;
    reg [TAG_WIDTH-1:0] pp0_tag, pp1_tag;

    // 满槽仅在本拍旧事务移走时接新事务；ready从输出向输入逐级传播。
    wire pp2_ready = !post_valid || post_ready;
    wire pp1_ready = !pp1_valid || pp2_ready;
    wire pp0_ready = !pp0_valid || pp1_ready;
    assign in_ready = pp0_ready;

    // PP0只做Bias加法，不把宽乘法串在此寄存器之前。
    function [39:0] add_bias_sat40;
        input signed [39:0] acc, bias;
        reg signed [40:0] sum;
        begin
            sum = {acc[39],acc} + {bias[39],bias};
            if (sum > 41'sd549755813887)
                add_bias_sat40 = 40'sh7f_ffff_ffff;
            else if (sum < -41'sd549755813888)
                add_bias_sat40 = 40'sh80_0000_0000;
            else
                add_bias_sat40 = sum[39:0];
        end
    endfunction

    // 对s>0：round_half_away(p/2^s)=(p+2^(s-1)-(p<0))>>>s。
    // 精确乘积先符号扩展到73位，再加舍入项，避免边界舍入溢出。
    function [15:0] round_shift_sat16;
        input signed [71:0] product;
        input [7:0] shift;
        input signed [7:0] zero_point;
        reg signed [72:0] extended_product, round_offset, shifted, requantized;
        begin
            extended_product = {product[71],product};
            round_offset = 73'sd0;
            if (shift == 0) begin
                shifted = extended_product;
            end else if (shift < 72) begin
                round_offset = (73'sd1 << (shift-1'b1)) -
                               (product[71] ? 73'sd1 : 73'sd0);
                shifted = (extended_product + round_offset) >>> shift;
            end else if (shift == 80) begin
                // 保留旧80-bit RTL在异常shift=80时的位级行为：正/零=-1，负=+1。
                // 部署文件shift实际0..42；此分支仅为完整UINT8输入域的旧版兼容，
                // 不是推荐量化公式。若改变异常参数政策，需另行统一Golden。
                shifted = product[71] ? 73'sd1 : -73'sd1;
            end else begin
                // INT40 x INT32的|p|<=2^70；72..79及81..255的旧输出均为0。
                shifted = 73'sd0;
            end
            requantized = shifted + {{65{zero_point[7]}},zero_point};
            if (requantized > 73'sd32767)
                round_shift_sat16 = 16'sh7fff;
            else if (requantized < -73'sd32768)
                round_shift_sat16 = 16'sh8000;
            else
                round_shift_sat16 = requantized[15:0];
        end
    endfunction

    integer lane;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pp0_valid <= 1'b0;
            pp1_valid <= 1'b0;
            post_valid <= 1'b0;
            pp0_biased <= 320'd0;
            pp0_mult <= 256'd0;
            pp0_shift <= 64'd0;
            pp0_zp <= 64'd0;
            pp1_product <= 576'd0;
            pp1_shift <= 64'd0;
            pp1_zp <= 64'd0;
            pp0_tag <= {TAG_WIDTH{1'b0}};
            pp1_tag <= {TAG_WIDTH{1'b0}};
            post_tag <= {TAG_WIDTH{1'b0}};
            post_data <= 128'd0;
        end else begin
            // PP2反压时，数据/Tag/valid一起保持，不能只停数据。
            if (pp2_ready) begin
                post_valid <= pp1_valid;
                if (pp1_valid) begin
                    for (lane=0; lane<8; lane=lane+1)
                        post_data[16*lane +: 16] <= round_shift_sat16(
                            $signed(pp1_product[72*lane +: 72]),
                            pp1_shift[8*lane +: 8], $signed(pp1_zp[8*lane +: 8]));
                    post_tag <= pp1_tag;
                end
            end
            // PP1乘法两端显式signed，完整72位结果随本笔shift/zp/Tag锁存。
            if (pp1_ready) begin
                pp1_valid <= pp0_valid;
                if (pp0_valid) begin
                    for (lane=0; lane<8; lane=lane+1)
                        pp1_product[72*lane +: 72] <=
                            $signed(pp0_biased[40*lane +: 40]) *
                            $signed(pp0_mult[32*lane +: 32]);
                    pp1_shift <= pp0_shift;
                    pp1_zp <= pp0_zp;
                    pp1_tag <= pp0_tag;
                end
            end
            // PP0握手时采样所有参数，不依赖外部参数在未来两拍继续稳定。
            if (pp0_ready) begin
                pp0_valid <= in_valid;
                if (in_valid) begin
                    for (lane=0; lane<8; lane=lane+1)
                        pp0_biased[40*lane +: 40] <= add_bias_sat40(
                            $signed(acc_data[40*lane +: 40]),
                            $signed(bias_data[64*lane +: 40]));
                    pp0_mult <= mult_data;
                    pp0_shift <= shift_data;
                    pp0_zp <= zero_point_data;
                    pp0_tag <= in_tag;
                end
            end
        end
    end
endmodule
