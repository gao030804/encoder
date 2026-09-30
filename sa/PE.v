// ============================================================================
// 中文阅读导引（当前实现）
// 单个权重固定乘加单元。每个CE有效上升沿：XOUT更新为XIN，PEOUT更新为PEIN+XIN×W。
// 默认输入和权重为signed INT8，乘积16 bit，符号扩展后累加到20 bit。
// PEIN来自上一行；此单元并不是把自身PEOUT反馈后反复累加。跨k_group累加由阵列外Accumulator完成。
// ============================================================================
//=============================================================================
// 模块名称：PE
// 功能：权重固定脉动阵列中的一个乘加处理单元。
//
// 每个有效时钟周期完成：
//   XOUT  <= XIN
//   PEOUT <= PEIN + XIN * W
//
// ACC_WIDTH独立于输入位宽。对于4个INT8乘积之和，20 bit可以覆盖
// 最坏情况并与上层Mesh接口保持一致。CE=0时数据和部分和同时保持。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 单PE完成signed INT8乘法，并把16-bit乘积符号扩展后加到INT20 PEIN。
// - XOUT把激活传给右侧PE，PEOUT把新部分和传给下一行；ce=0时全部寄存器保持。
// - 本模块只计算一个k_group内的部分和，跨k_group的INT32累加由accumulator_8lane完成。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：权重固定脉动阵列处理单元。
// 关键变量/接口：XIN/XOUT横向传激活；PEIN/PEOUT纵向传INT20部分和；W为本PE固定INT8权重。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
// W8A16合同：Activation=INT16，Weight=INT8，单乘积INT24，4路部分和INT26。
module PE #(
    parameter ACT_WIDTH  = 16,
    parameter ACC_WIDTH  = 26,
    parameter WGT_WIDTH  = 8
) (
    input                                       CLK,
    input                                       RSTn,
    input                                       CE,
    input       signed [WGT_WIDTH-1:0]          W,
    input       signed [ACT_WIDTH-1:0]          XIN,
    input       signed [ACC_WIDTH-1:0]          PEIN,
    output reg  signed [ACT_WIDTH-1:0]          XOUT,
    output reg  signed [ACC_WIDTH-1:0]          PEOUT
);

// 组合乘法：INT8×INT8产生16-bit signed乘积。
wire signed [ACT_WIDTH+WGT_WIDTH-1:0] product = $signed(XIN) * $signed(W);

// 乘积先符号扩展到部分和宽度，再参与加法，避免负数扩展错误。
wire signed [ACC_WIDTH-1:0] product_ext =
    {{(ACC_WIDTH-ACT_WIDTH-WGT_WIDTH){product[ACT_WIDTH+WGT_WIDTH-1]}}, product};

// PE内所有数据均为时序输出。CE是整条Mesh的统一流水使能；CE为0时，
// 激活和部分和必须一起保持，否则相邻PE的计算拍会发生错位。
// 时序逻辑：在时钟沿更新XOUT、PEOUT；复位分支负责恢复确定的空闲状态。
always @(posedge CLK or negedge RSTn) begin
    if (!RSTn) begin
        XOUT  <= {ACT_WIDTH{1'b0}};
        PEOUT <= {ACC_WIDTH{1'b0}};
    end else if (CE) begin
        XOUT  <= XIN;
        PEOUT <= $signed(PEIN) + $signed(product_ext);
    end
end

endmodule
