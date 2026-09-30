// ============================================================================
// 中文阅读导引（当前实现）
// 8路INT8查表激活，正半轴旁路，负半轴使用LUT；每路都需要同时查询能力。
// LUT必须按部署Scale生成，不能把示例Scale当作每层固定Scale。是否启用由层配置决定。
// ============================================================================
//=============================================================================
// 模块：elu_8lane
// 功能：每拍并行处理 8 个 signed INT8 ELU 激活值。
//
// ELU(alpha=1)：
//   y = x,          x >= 0
//   y = exp(x) - 1, x <  0
//
// 硬件实现：
//   1. 正数路径直接旁路；
//   2. 负数路径查询 128 项 × 8 bit LUT；
//   3. 为支持每拍 8 个不同地址，物理上复制 8 份相同 LUT；
//   4. 配置端口每次写入会广播到全部 8 份 LUT；
//   5. 输出端带一拍弹性寄存器，支持 valid/ready 反压。
//
// LUT 地址约定：
//   地址 0   对应输入 -1；
//   地址 1   对应输入 -2；
//   ...
//   地址 127 对应输入 -128。
//
// 量化要求：正数直接旁路要求 ELU 输入、输出使用相同的 scale 和
// zero_point=0。负半轴 LUT 必须由模型离线校准后生成，并在 elu_enable
// 拉高之前写完全部 128 项。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 8路负数分别访问复制的128项LUT，正数直接旁路；复制LUT用于获得8个并行读口。
// - relu_enable优先于elu_enable；ReLU把负数置0，ELU按abs(x)-1形成LUT地址。
// - 配置LUT时数据通路必须空闲，避免计算过程中修改查表内容。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：八路ReLU/ELU激活单元。
// 关键变量/接口：ReLU将负数置零；ELU负半轴查询128项LUT；输出遵守valid/ready反压。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module elu_8lane (
    input                                       clk,
    input                                       rst_n,

    // LUT配置接口。配置应在数据流水空闲时进行。
    input                                       cfg_valid,
    output                                      cfg_ready,
    input              [6:0]                    cfg_addr,
    input              [7:0]                    cfg_wdata,

    // elu_enable=0 时，64-bit输入数据原样旁路。
    input                                       elu_enable,

    // ReLU 与 ELU 互斥；若同时置位，ReLU 优先，负数直接输出 0。
    input                                       relu_enable,
    input                                       in_valid,
    output                                      in_ready,
    input              [127:0]                  in_data,
    output reg                                  out_valid,
    input                                       out_ready,
    output reg         [127:0]                  out_data
);

// 8份内容相同的负半轴LUT，提供8个独立异步读端口。
// 不在复位时清零，避免综合出大规模复位网络。
reg signed [7:0] lut0 [0:127];
reg signed [7:0] lut1 [0:127];
reg signed [7:0] lut2 [0:127];
reg signed [7:0] lut3 [0:127];
reg signed [7:0] lut4 [0:127];
reg signed [7:0] lut5 [0:127];
reg signed [7:0] lut6 [0:127];
reg signed [7:0] lut7 [0:127];
wire [15:0] lane0 = in_data[ 15:  0];
wire [15:0] lane1 = in_data[ 31: 16];
wire [15:0] lane2 = in_data[ 47: 32];
wire [15:0] lane3 = in_data[ 63: 48];
wire [15:0] lane4 = in_data[ 79: 64];
wire [15:0] lane5 = in_data[ 95: 80];
wire [15:0] lane6 = in_data[111: 96];
wire [15:0] lane7 = in_data[127:112];

// 对负数二进制补码x，~x的低7位恰好等于abs(x)-1：
// -1 -> 0，-2 -> 1，-128 -> 127。
wire [6:0] index0 = ~lane0[6:0];
wire [6:0] index1 = ~lane1[6:0];
wire [6:0] index2 = ~lane2[6:0];
wire [6:0] index3 = ~lane3[6:0];
wire [6:0] index4 = ~lane4[6:0];
wire [6:0] index5 = ~lane5[6:0];
wire [6:0] index6 = ~lane6[6:0];
wire [6:0] index7 = ~lane7[6:0];
wire [15:0] elu0 = lane0[15] ? {{8{lut0[index0][7]}},lut0[index0]} : lane0;
wire [15:0] elu1 = lane1[15] ? {{8{lut1[index1][7]}},lut1[index1]} : lane1;
wire [15:0] elu2 = lane2[15] ? {{8{lut2[index2][7]}},lut2[index2]} : lane2;
wire [15:0] elu3 = lane3[15] ? {{8{lut3[index3][7]}},lut3[index3]} : lane3;
wire [15:0] elu4 = lane4[15] ? {{8{lut4[index4][7]}},lut4[index4]} : lane4;
wire [15:0] elu5 = lane5[15] ? {{8{lut5[index5][7]}},lut5[index5]} : lane5;
wire [15:0] elu6 = lane6[15] ? {{8{lut6[index6][7]}},lut6[index6]} : lane6;
wire [15:0] elu7 = lane7[15] ? {{8{lut7[index7][7]}},lut7[index7]} : lane7;
wire [127:0] elu_result = {
elu7, elu6, elu5, elu4,
elu3, elu2, elu1, elu0
};
wire [127:0] relu_result = {
lane7[15] ? 16'd0 : lane7, lane6[15] ? 16'd0 : lane6,
lane5[15] ? 16'd0 : lane5, lane4[15] ? 16'd0 : lane4,
lane3[15] ? 16'd0 : lane3, lane2[15] ? 16'd0 : lane2,
lane1[15] ? 16'd0 : lane1, lane0[15] ? 16'd0 : lane0
};

// 一项输出缓冲：空闲或本拍即将被下游接收时，可以接收新输入。
// 连续赋值：组合生成in_ready及其相邻接口信号，表达握手、选择或地址关系。
assign in_ready  = !out_valid || out_ready;

// 禁止配置写和数据处理重叠，避免查表时修改LUT内容。
// 连续赋值：组合生成cfg_ready及其相邻接口信号，表达握手、选择或地址关系。
assign cfg_ready = !in_valid && !out_valid;

// 配置写入广播到8份LUT，保证所有lane使用相同映射关系。

// 时序逻辑：在时钟沿更新out_valid、out_data；复位分支负责恢复确定的空闲状态。
always @(posedge clk) begin
    if (cfg_valid && cfg_ready) begin
        lut0[cfg_addr] <= cfg_wdata;
        lut1[cfg_addr] <= cfg_wdata;
        lut2[cfg_addr] <= cfg_wdata;
        lut3[cfg_addr] <= cfg_wdata;
        lut4[cfg_addr] <= cfg_wdata;
        lut5[cfg_addr] <= cfg_wdata;
        lut6[cfg_addr] <= cfg_wdata;
        lut7[cfg_addr] <= cfg_wdata;
    end
end

// ELU数据流水。发生反压时out_valid和out_data均保持不变。

// 时序逻辑：在时钟沿更新out_valid、out_data；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        out_valid <= 1'b0;
        out_data  <= 128'd0;
    end else if (in_ready) begin
        out_valid <= in_valid;
        if (in_valid)
            out_data <= relu_enable ? relu_result :
            (elu_enable ? elu_result : in_data);
    end
end

endmodule
