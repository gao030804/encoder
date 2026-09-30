//=============================================================================
// 模块：conv1d_config_latch
// 功能：锁存一次 Conv1d 运算所需的全部配置。
//
// latch_en 拉高的时钟沿会同时保存所有 cfg_* 输入。之后即使软件侧修改
// cfg_*，当前层仍使用这里保存的配置，避免一层计算过程中参数发生变化。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 在IDLE接受start的同一拍锁存全部cfg字段，使一个卷积事务内配置保持不变。
// - 地址以Byte为单位；长度表示time维；k_groups和n_groups由Layer ROM预先计算。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：卷积配置锁存器。
// 关键变量/接口：start握手时锁存Cin/Cout/K/步幅/膨胀/长度/基址，保证单层计算期间配置稳定。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module conv1d_config_latch (
    input                                       clk,
    input                                       rst_n,
    input                                       latch_en,
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

    // 1：本层输出写回前经过 ELU；0：旁路，适用于无激活函数的卷积层。
    input                                       cfg_elu_enable,
    input                                       cfg_relu_enable,
    output reg         [31:0]                   input_base,
    output reg         [31:0]                   output_base,
    output reg         [31:0]                   weight_base,
    output reg         [8:0]                    cin,
    output reg         [8:0]                    cout,
    output reg         [4:0]                    kernel,
    output reg         [3:0]                    stride,
    output reg         [3:0]                    dilation,
    output reg         [7:0]                    left_pad,
    output reg         [8:0]                    input_length,
    output reg         [8:0]                    output_length,
    output reg         [11:0]                   k_total,
    output reg         [9:0]                    k_groups,
    output reg         [5:0]                    n_groups,
    output reg                                  elu_enable,
    output reg                                  relu_enable
);

// 时序逻辑：在时钟沿更新input_base、output_base、weight_base、cin、cout、kernel；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin

        // 这里只复位配置寄存器，不涉及 SRAM 数据内容。
        input_base    <= 32'd0;
        output_base   <= 32'd0;
        weight_base   <= 32'd0;
        cin           <= 9'd0;
        cout          <= 9'd0;
        kernel        <= 5'd0;
        stride        <= 4'd0;
        dilation      <= 4'd0;
        left_pad      <= 8'd0;
        input_length  <= 9'd0;
        output_length <= 9'd0;
        k_total       <= 12'd0;
        k_groups      <= 10'd0;
        n_groups      <= 6'd0;
        elu_enable    <= 1'b0;
        relu_enable   <= 1'b0;
    end else if (latch_en) begin

        // 原子锁存：所有配置字段在同一个时钟沿更新。
        input_base    <= cfg_input_base;
        output_base   <= cfg_output_base;
        weight_base   <= cfg_weight_base;
        cin           <= cfg_cin;
        cout          <= cfg_cout;
        kernel        <= cfg_kernel;
        stride        <= cfg_stride;
        dilation      <= cfg_dilation;
        left_pad      <= cfg_left_pad;
        input_length  <= cfg_input_length;
        output_length <= cfg_output_length;
        k_total       <= cfg_k_total;
        k_groups      <= cfg_k_groups;
        n_groups      <= cfg_n_groups;
        elu_enable    <= cfg_elu_enable;
        relu_enable   <= cfg_relu_enable;
    end
end

endmodule
