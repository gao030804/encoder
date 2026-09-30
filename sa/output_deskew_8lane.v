// ============================================================================
// 中文阅读导引（当前实现）
// 对8列不同延迟的部分和进行补偿，使同一个m的8个输出通道在同拍出现。
// 延迟数据时同时延迟valid和标签；不能只移动数据而丢失其对应的输出位置。
// ============================================================================
//=============================================================================
// 模块：output_deskew_8lane
// 功能：补偿 8 个输出列不同的到达时间，将同一个 m_tag 的结果重新对齐。
//
// 默认 D0..D7=7..0，对应理想逐 PE 波前。每列的 INT20 数据、valid 和
// m_tag 同步延迟；只有 8 列都有效时 out_valid 才有效。ce=0 时全部保持。
// 实际延迟必须结合真实 2x2 Tile 组合逻辑和 Tile 边界寄存器重新配置。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 补偿8个输出列不同的阵列延迟，使同一m_tag的8路INT20部分和同时输出。
// - valid和tag必须经过与数据完全相同的ce延迟链，否则会写入错误的m地址。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：8路部分和对齐器。
// 关键变量/接口：补偿各列传播延迟，使同一m_tag的8路INT20部分和同周期送入Accumulator。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module output_deskew_8lane #(
    parameter M_TAG_WIDTH = 9,
    parameter PSUM_WIDTH = 26,
    parameter D0 = 7,
    parameter D1 = 6,
    parameter D2 = 5,
    parameter D3 = 4,
    parameter D4 = 3,
    parameter D5 = 2,
    parameter D6 = 1,
    parameter D7 = 0
) (
    input                                       clk,
    input                                       rst_n,
    input                                       ce,
    input              [7:0]                    in_valid,
    input              [8*PSUM_WIDTH-1:0]       in_data,
    input              [8*M_TAG_WIDTH-1:0]      in_m_tag,
    output                                      out_valid,
    output             [8*PSUM_WIDTH-1:0]       out_data,
    output             [M_TAG_WIDTH-1:0]        out_m_tag
);

reg [PSUM_WIDTH-1:0] q0 [0:D0];
reg [PSUM_WIDTH-1:0] q1 [0:D1];
reg [PSUM_WIDTH-1:0] q2 [0:D2];
reg [PSUM_WIDTH-1:0] q3 [0:D3];
reg [PSUM_WIDTH-1:0] q4 [0:D4];
reg [PSUM_WIDTH-1:0] q5 [0:D5];
reg [PSUM_WIDTH-1:0] q6 [0:D6];
reg [PSUM_WIDTH-1:0] q7 [0:D7];
reg v0 [0:D0];
reg v1 [0:D1];
reg v2 [0:D2];
reg v3 [0:D3];
reg v4 [0:D4];
reg v5 [0:D5];
reg v6 [0:D6];
reg v7 [0:D7];
reg [M_TAG_WIDTH-1:0] t0 [0:D0];
reg [M_TAG_WIDTH-1:0] t1 [0:D1];
reg [M_TAG_WIDTH-1:0] t2 [0:D2];
reg [M_TAG_WIDTH-1:0] t3 [0:D3];
reg [M_TAG_WIDTH-1:0] t4 [0:D4];
reg [M_TAG_WIDTH-1:0] t5 [0:D5];
reg [M_TAG_WIDTH-1:0] t6 [0:D6];
reg [M_TAG_WIDTH-1:0] t7 [0:D7];
integer i;
// 连续赋值：组合生成out_data及其相邻接口信号，表达握手、选择或地址关系。
assign out_data = {
q7[D7], q6[D6], q5[D5], q4[D4],
q3[D3], q2[D2], q1[D1], q0[D0]
};
// 连续赋值：组合生成out_valid及其相邻接口信号，表达握手、选择或地址关系。
assign out_valid = v0[D0] & v1[D1] & v2[D2] & v3[D3] &
    v4[D4] & v5[D5] & v6[D6] & v7[D7];

// 对齐完成后各列 tag 应相同，因此选择 lane0 的 tag 作为公共输出 tag。
// 连续赋值：组合生成out_m_tag及其相邻接口信号，表达握手、选择或地址关系。
assign out_m_tag = t0[D0];

// 时序逻辑：在时钟沿更新i；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin

        // reset 必须清空全部 valid，避免复位后输出伪结果。
        for (i = 0; i <= D0; i = i + 1) begin
            q0[i] <= {PSUM_WIDTH{1'b0}};
            v0[i] <= 1'b0;
            t0[i] <= {M_TAG_WIDTH{1'b0}};
        end
        for (i = 0; i <= D1; i = i + 1) begin
            q1[i] <= {PSUM_WIDTH{1'b0}};
            v1[i] <= 1'b0;
            t1[i] <= {M_TAG_WIDTH{1'b0}};
        end
        for (i = 0; i <= D2; i = i + 1) begin
            q2[i] <= {PSUM_WIDTH{1'b0}};
            v2[i] <= 1'b0;
            t2[i] <= {M_TAG_WIDTH{1'b0}};
        end
        for (i = 0; i <= D3; i = i + 1) begin
            q3[i] <= {PSUM_WIDTH{1'b0}};
            v3[i] <= 1'b0;
            t3[i] <= {M_TAG_WIDTH{1'b0}};
        end
        for (i = 0; i <= D4; i = i + 1) begin
            q4[i] <= {PSUM_WIDTH{1'b0}};
            v4[i] <= 1'b0;
            t4[i] <= {M_TAG_WIDTH{1'b0}};
        end
        for (i = 0; i <= D5; i = i + 1) begin
            q5[i] <= {PSUM_WIDTH{1'b0}};
            v5[i] <= 1'b0;
            t5[i] <= {M_TAG_WIDTH{1'b0}};
        end
        for (i = 0; i <= D6; i = i + 1) begin
            q6[i] <= {PSUM_WIDTH{1'b0}};
            v6[i] <= 1'b0;
            t6[i] <= {M_TAG_WIDTH{1'b0}};
        end
        for (i = 0; i <= D7; i = i + 1) begin
            q7[i] <= {PSUM_WIDTH{1'b0}};
            v7[i] <= 1'b0;
            t7[i] <= {M_TAG_WIDTH{1'b0}};
        end
    end else if (ce) begin

        // 各列先进入自己的第 0 级，再按独立 D 参数向后移动。
        q0[0] <= in_data[PSUM_WIDTH-1:0];
        q1[0] <= in_data[2*PSUM_WIDTH-1:PSUM_WIDTH];
        q2[0] <= in_data[3*PSUM_WIDTH-1:2*PSUM_WIDTH];
        q3[0] <= in_data[4*PSUM_WIDTH-1:3*PSUM_WIDTH];
        q4[0] <= in_data[5*PSUM_WIDTH-1:4*PSUM_WIDTH];
        q5[0] <= in_data[6*PSUM_WIDTH-1:5*PSUM_WIDTH];
        q6[0] <= in_data[7*PSUM_WIDTH-1:6*PSUM_WIDTH];
        q7[0] <= in_data[8*PSUM_WIDTH-1:7*PSUM_WIDTH];
        v0[0] <= in_valid[0];
        v1[0] <= in_valid[1];
        v2[0] <= in_valid[2];
        v3[0] <= in_valid[3];
        v4[0] <= in_valid[4];
        v5[0] <= in_valid[5];
        v6[0] <= in_valid[6];
        v7[0] <= in_valid[7];
        t0[0] <= in_m_tag[M_TAG_WIDTH-1:0];
        t1[0] <= in_m_tag[2*M_TAG_WIDTH-1:M_TAG_WIDTH];
        t2[0] <= in_m_tag[3*M_TAG_WIDTH-1:2*M_TAG_WIDTH];
        t3[0] <= in_m_tag[4*M_TAG_WIDTH-1:3*M_TAG_WIDTH];
        t4[0] <= in_m_tag[5*M_TAG_WIDTH-1:4*M_TAG_WIDTH];
        t5[0] <= in_m_tag[6*M_TAG_WIDTH-1:5*M_TAG_WIDTH];
        t6[0] <= in_m_tag[7*M_TAG_WIDTH-1:6*M_TAG_WIDTH];
        t7[0] <= in_m_tag[8*M_TAG_WIDTH-1:7*M_TAG_WIDTH];
        for (i = 1; i <= D0; i = i + 1) begin
            q0[i] <= q0[i-1];
            v0[i] <= v0[i-1];
            t0[i] <= t0[i-1];
        end
        for (i = 1; i <= D1; i = i + 1) begin
            q1[i] <= q1[i-1];
            v1[i] <= v1[i-1];
            t1[i] <= t1[i-1];
        end
        for (i = 1; i <= D2; i = i + 1) begin
            q2[i] <= q2[i-1];
            v2[i] <= v2[i-1];
            t2[i] <= t2[i-1];
        end
        for (i = 1; i <= D3; i = i + 1) begin
            q3[i] <= q3[i-1];
            v3[i] <= v3[i-1];
            t3[i] <= t3[i-1];
        end
        for (i = 1; i <= D4; i = i + 1) begin
            q4[i] <= q4[i-1];
            v4[i] <= v4[i-1];
            t4[i] <= t4[i-1];
        end
        for (i = 1; i <= D5; i = i + 1) begin
            q5[i] <= q5[i-1];
            v5[i] <= v5[i-1];
            t5[i] <= t5[i-1];
        end
        for (i = 1; i <= D6; i = i + 1) begin
            q6[i] <= q6[i-1];
            v6[i] <= v6[i-1];
            t6[i] <= t6[i-1];
        end
        for (i = 1; i <= D7; i = i + 1) begin
            q7[i] <= q7[i-1];
            v7[i] <= v7[i-1];
            t7[i] <= t7[i-1];
        end
    end
end

endmodule
