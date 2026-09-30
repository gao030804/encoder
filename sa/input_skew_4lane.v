// ============================================================================
// 中文阅读导引（当前实现）
// 四路激活分别延迟0/1/2/3个使能周期，使规约项与阵列内部分和到达时刻匹配。
// ce暂停时按使能周期计算延迟，不能按绝对时钟数判断波形错位。
// ============================================================================
//=============================================================================
// 模块：input_skew_4lane
// 功能：为 4 个输入行加入可配置的逐行延迟，形成脉动阵列输入波前。
//
// 默认延迟为 0/1/2/3。每个 lane 的 data、valid 和 m_tag 使用同样深度
// 的移位寄存器，防止数据与标签错位。ce=0 时所有寄存器保持不变。
// 注意：默认值只适用于理想逐 PE 流水，真实 Tile 延迟需要重新标定。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 第0/1/2/3路分别延迟0/1/2/3个有效ce周期，使数据沿脉动阵列对角线进入。
// - ce=0时数据和valid必须同时冻结，不能把墙钟周期误当作阵列推进周期。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：4路激活错拍器。
// 关键变量/接口：lane0~3依次延迟0~3拍，使同一k_group的4个激活沿阵列对角线到达PE。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module input_skew_4lane #(
    parameter M_TAG_WIDTH = 9,
    parameter ACT_WIDTH = 16,
    parameter DELAY0 = 0,
    parameter DELAY1 = 1,
    parameter DELAY2 = 2,
    parameter DELAY3 = 3
) (
    input                                       clk,
    input                                       rst_n,
    input                                       ce,
    input                                       in_valid,
    input              [4*ACT_WIDTH-1:0]        in_data,
    input              [M_TAG_WIDTH-1:0]        in_m_tag,
    output             [3:0]                    out_valid,
    output             [4*ACT_WIDTH-1:0]        out_data,
    output             [4*M_TAG_WIDTH-1:0]      out_m_tag
);

reg [ACT_WIDTH-1:0] d0 [0:DELAY0];
reg [ACT_WIDTH-1:0] d1 [0:DELAY1];
reg [ACT_WIDTH-1:0] d2 [0:DELAY2];
reg [ACT_WIDTH-1:0] d3 [0:DELAY3];
reg v0 [0:DELAY0];
reg v1 [0:DELAY1];
reg v2 [0:DELAY2];
reg v3 [0:DELAY3];
reg [M_TAG_WIDTH-1:0] t0 [0:DELAY0];
reg [M_TAG_WIDTH-1:0] t1 [0:DELAY1];
reg [M_TAG_WIDTH-1:0] t2 [0:DELAY2];
reg [M_TAG_WIDTH-1:0] t3 [0:DELAY3];
integer i;
// 连续赋值：组合生成out_data及其相邻接口信号，表达握手、选择或地址关系。
assign out_data = {d3[DELAY3], d2[DELAY2], d1[DELAY1], d0[DELAY0]};
assign out_valid = {v3[DELAY3], v2[DELAY2], v1[DELAY1], v0[DELAY0]};
// 连续赋值：组合生成out_m_tag及其相邻接口信号，表达握手、选择或地址关系。
assign out_m_tag = {t3[DELAY3], t2[DELAY2], t1[DELAY1], t0[DELAY0]};

// 时序逻辑：在时钟沿更新i；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin

        // 数据寄存器可复位为 0；更关键的是将所有 valid 清零。
        for (i = 0; i <= DELAY0; i = i + 1) begin
            d0[i] <= {ACT_WIDTH{1'b0}};
            v0[i] <= 1'b0;
            t0[i] <= {M_TAG_WIDTH{1'b0}};
        end
        for (i = 0; i <= DELAY1; i = i + 1) begin
            d1[i] <= {ACT_WIDTH{1'b0}};
            v1[i] <= 1'b0;
            t1[i] <= {M_TAG_WIDTH{1'b0}};
        end
        for (i = 0; i <= DELAY2; i = i + 1) begin
            d2[i] <= {ACT_WIDTH{1'b0}};
            v2[i] <= 1'b0;
            t2[i] <= {M_TAG_WIDTH{1'b0}};
        end
        for (i = 0; i <= DELAY3; i = i + 1) begin
            d3[i] <= {ACT_WIDTH{1'b0}};
            v3[i] <= 1'b0;
            t3[i] <= {M_TAG_WIDTH{1'b0}};
        end
    end else if (ce) begin

        // 第 0 级接收新向量，各 lane 之后按自己的 DELAY 参数移动。
        d0[0] <= in_data[ACT_WIDTH-1:0];
        v0[0] <= in_valid;
        t0[0] <= in_m_tag;
        d1[0] <= in_data[2*ACT_WIDTH-1:ACT_WIDTH];
        v1[0] <= in_valid;
        t1[0] <= in_m_tag;
        d2[0] <= in_data[3*ACT_WIDTH-1:2*ACT_WIDTH];
        v2[0] <= in_valid;
        t2[0] <= in_m_tag;
        d3[0] <= in_data[4*ACT_WIDTH-1:3*ACT_WIDTH];
        v3[0] <= in_valid;
        t3[0] <= in_m_tag;
        for (i = 1; i <= DELAY0; i = i + 1) begin
            d0[i] <= d0[i-1];
            v0[i] <= v0[i-1];
            t0[i] <= t0[i-1];
        end
        for (i = 1; i <= DELAY1; i = i + 1) begin
            d1[i] <= d1[i-1];
            v1[i] <= v1[i-1];
            t1[i] <= t1[i-1];
        end
        for (i = 1; i <= DELAY2; i = i + 1) begin
            d2[i] <= d2[i-1];
            v2[i] <= v2[i-1];
            t2[i] <= t2[i-1];
        end
        for (i = 1; i <= DELAY3; i = i + 1) begin
            d3[i] <= d3[i-1];
            v3[i] <= v3[i-1];
            t3[i] <= t3[i-1];
        end
    end
end

endmodule
