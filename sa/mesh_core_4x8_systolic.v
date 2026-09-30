// ============================================================================
// 中文阅读导引（当前实现）
// 计算阵列封装：输入错位 → 4×8 PE → 输出对齐。
// Active和Shadow各256 bit，共64 Byte；Shadow加载不会改变当前PE所见权重。
// weight_swap必须在当前权重的所有在途计算结束后使用，由上层保证该边界。
// ce=0时权重、数据和valid/tag流水共同保持。RAW_DELAY记录各列结果延迟。
// ============================================================================
//=============================================================================
// 模块名称：mesh_core_4x8_systolic
// 功能：把Systolic_Array适配成上层Mesh使用的向量接口。
//
// 适配内容：
//   1. weight_load时锁存一个4×8 INT8权重块；
//   2. 4路激活执行0/1/2/3拍输入错位；
//   3. 跟踪每一列不同的阵列延迟；
//   4. 8列结果执行7/6/.../0拍输出对齐；
//   5. ce=0时权重、数据、valid和tag统一冻结。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 保存Active/Shadow两组256-bit权重，并将4路INT8激活送入实际Systolic_Array。
// - 每列对应一个输出通道；每行对应当前k_group中的一个reduction lane。
// - ce统一控制数据、valid和tag流水，保证反压时所有信号停在相同计算位置。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：4×8 Mesh核心。
// 关键变量/接口：保存Active/Shadow两套256-bit权重寄存器；ce同时冻结数据、valid和tag流水。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module mesh_core_4x8_systolic #(
    parameter M_TAG_WIDTH = 9,
    parameter ACT_WIDTH = 16,
    parameter PSUM_WIDTH = 26
) (
    input                                       clk,
    input                                       rst_n,
    input                                       ce,

    // weight_load直接更新当前活动寄存器，用于首个k_group或无预取回退路径。
    input                                       weight_load,

    // 当前k_group计算期间，可提前写入下一组权重；weight_swap在边界切换。
    input                                       weight_shadow_load,
    input                                       weight_swap,
    input              [255:0]                  weight_data,
    input                                       act_valid,
    input              [4*ACT_WIDTH-1:0]        act_data,
    input              [M_TAG_WIDTH-1:0]        act_m_tag,
    output                                      psum_valid,
    output             [8*PSUM_WIDTH-1:0]       psum_data,
    output             [M_TAG_WIDTH-1:0]        psum_m_tag
);

localparam RAW_DELAY0 = 4;
localparam RAW_DELAY1 = 5;
localparam RAW_DELAY2 = 6;
localparam RAW_DELAY3 = 7;
localparam RAW_DELAY4 = 8;
localparam RAW_DELAY5 = 9;
localparam RAW_DELAY6 = 10;
localparam RAW_DELAY7 = 11;
reg [255:0] weight_q;
reg [255:0] weight_shadow_q;
reg [11:0] valid_pipe;
reg [M_TAG_WIDTH-1:0] tag_pipe [0:11];
integer i;
wire [3:0] skew_valid;
wire [4*ACT_WIDTH-1:0] skew_data;
wire [4*M_TAG_WIDTH-1:0] skew_tag;
wire [8*PSUM_WIDTH-1:0] raw_psum;
wire [7:0] raw_valid;
wire [8*M_TAG_WIDTH-1:0] raw_tag;

// 时序逻辑：在时钟沿更新weight_q、weight_shadow_q、valid_pipe；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        weight_q  <= 256'd0;
        weight_shadow_q <= 256'd0;
        valid_pipe <= 12'd0;
        for (i = 0; i < 12; i = i + 1) begin
            tag_pipe[i] <= {M_TAG_WIDTH{1'b0}};
        end
    end else if (ce) begin
        if (weight_load) begin
            weight_q <= weight_data;
        end
        if (weight_shadow_load) begin
            weight_shadow_q <= weight_data;
        end

        // 非阻塞赋值保证同一周期Shadow装载与Swap同时出现时，Active取得旧Shadow。
        // 正常控制逻辑不会让二者同周期出现，这里仍保留确定的硬件语义。
        if (weight_swap) begin
            weight_q <= weight_shadow_q;
        end
        valid_pipe[0] <= act_valid;
        tag_pipe[0]   <= act_m_tag;
        for (i = 1; i < 12; i = i + 1) begin
            valid_pipe[i] <= valid_pipe[i-1];
            tag_pipe[i]   <= tag_pipe[i-1];
        end
    end
end

input_skew_4lane #(
    .M_TAG_WIDTH                 (M_TAG_WIDTH),
    .ACT_WIDTH                   (ACT_WIDTH),
    .DELAY0                      (0),
    .DELAY1                      (1),
    .DELAY2                      (2),
    .DELAY3                      (3)
) u_input_skew (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .ce                          (ce),
    .in_valid                    (act_valid),
    .in_data                     (act_data),
    .in_m_tag                    (act_m_tag),
    .out_valid                   (skew_valid),
    .out_data                    (skew_data),
    .out_m_tag                   (skew_tag)
);

Systolic_Array #(
    .ACT_WIDTH                   (ACT_WIDTH),
    .WGT_WIDTH                   (8),
    .ACC_WIDTH                   (PSUM_WIDTH)
) u_systolic_array (
    .CLK                         (clk),
    .RSTn                        (rst_n),
    .CE                          (ce),
    .WEIGHT_DATA                 (weight_q),
    .XIN_DATA                    (skew_data),
    .YOUT_DATA                   (raw_psum)
);

// 第n列的最终部分和相对输入向量延迟4+n拍。
// 连续赋值：组合生成raw_valid及其相邻接口信号，表达握手、选择或地址关系。
assign raw_valid[0] = valid_pipe[RAW_DELAY0];
assign raw_valid[1] = valid_pipe[RAW_DELAY1];
// 连续赋值：组合生成raw_valid及其相邻接口信号，表达握手、选择或地址关系。
assign raw_valid[2] = valid_pipe[RAW_DELAY2];
assign raw_valid[3] = valid_pipe[RAW_DELAY3];
// 连续赋值：组合生成raw_valid及其相邻接口信号，表达握手、选择或地址关系。
assign raw_valid[4] = valid_pipe[RAW_DELAY4];
assign raw_valid[5] = valid_pipe[RAW_DELAY5];
// 连续赋值：组合生成raw_valid及其相邻接口信号，表达握手、选择或地址关系。
assign raw_valid[6] = valid_pipe[RAW_DELAY6];
assign raw_valid[7] = valid_pipe[RAW_DELAY7];
// 连续赋值：组合生成raw_tag及其相邻接口信号，表达握手、选择或地址关系。
assign raw_tag[M_TAG_WIDTH-1:0] = tag_pipe[RAW_DELAY0];
assign raw_tag[2*M_TAG_WIDTH-1:M_TAG_WIDTH] = tag_pipe[RAW_DELAY1];
// 连续赋值：组合生成raw_tag及其相邻接口信号，表达握手、选择或地址关系。
assign raw_tag[3*M_TAG_WIDTH-1:2*M_TAG_WIDTH] = tag_pipe[RAW_DELAY2];
assign raw_tag[4*M_TAG_WIDTH-1:3*M_TAG_WIDTH] = tag_pipe[RAW_DELAY3];
// 连续赋值：组合生成raw_tag及其相邻接口信号，表达握手、选择或地址关系。
assign raw_tag[5*M_TAG_WIDTH-1:4*M_TAG_WIDTH] = tag_pipe[RAW_DELAY4];
assign raw_tag[6*M_TAG_WIDTH-1:5*M_TAG_WIDTH] = tag_pipe[RAW_DELAY5];
// 连续赋值：组合生成raw_tag及其相邻接口信号，表达握手、选择或地址关系。
assign raw_tag[7*M_TAG_WIDTH-1:6*M_TAG_WIDTH] = tag_pipe[RAW_DELAY6];
assign raw_tag[8*M_TAG_WIDTH-1:7*M_TAG_WIDTH] = tag_pipe[RAW_DELAY7];
output_deskew_8lane #(
    .M_TAG_WIDTH                 (M_TAG_WIDTH),
    .PSUM_WIDTH                  (PSUM_WIDTH),
    .D0                          (7),
    .D1                          (6),
    .D2                          (5),
    .D3                          (4),
    .D4                          (3),
    .D5                          (2),
    .D6                          (1),
    .D7                          (0)
) u_output_deskew (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .ce                          (ce),
    .in_valid                    (raw_valid),
    .in_data                     (raw_psum),
    .in_m_tag                    (raw_tag),
    .out_valid                   (psum_valid),
    .out_data                    (psum_data),
    .out_m_tag                   (psum_m_tag)
);

endmodule
