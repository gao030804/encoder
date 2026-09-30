// ============================================================================
// 中文阅读导引（当前实现）
// 纯组合地址计算，无clk，也不直接发起SRAM握手。上层在请求被接收时推进m/k_group。
// r=4*k_group+lane；kernel_index=r/Cin；channel_index=r%Cin。
// time=m*stride-left_pad+kernel_index*dilation；byte_addr=input_base+time*Cin+channel_index。
// 负time、越过输入长度或K尾部均通过valid_mask置零。mask=0时对应地址不应被作为有效激活读取。
// 合法Cin以本模块case列表为准，并非任意2的幂都受到支持。
// ============================================================================
//=============================================================================
// 模块：conv1d_agu
// 功能：为一个 m、一个 k_group 生成 4 路 Activation SRAM 地址。
//
// 展平后的 K 维索引：r = 4*k_group + lane
// kernel_index  = r / Cin
// channel_index = r % Cin
// time_index    = m*stride - left_pad + kernel_index*dilation
//
// Cin 只允许 2 的幂，因此除法和取模分别用右移及按位与实现，避免综合出
// 通用除法器。Padding 和 K 尾部不会真的访问 SRAM，而是通过 valid_mask
// 标记为无效，后级将相应激活值强制变为 0。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 组合地址公式：r=4*k_group+lane，kernel_index=r/Cin，channel_index=r%Cin。
// - time=m*stride-left_pad+kernel_index*dilation，addr=input_base+time*Cin+channel_index。
// - time越界或r>=Cin*K时清除对应valid_mask；mask=0的地址值不得用于SRAM访问。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：4路Activation地址生成器。
// 关键变量/接口：r=4*k_group+lane；time=m*stride-pad+(r/Cin)*dilation；越界lane由valid_mask屏蔽。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module conv1d_agu #(
    parameter ADDR_WIDTH = 32,                    // SRAM 地址宽度
    parameter M_TAG_WIDTH = 9                     // 输出时间位置 m 的位宽
) (
    input              [ADDR_WIDTH-1:0]         input_base,       // Activation SRAM 基地址
    input              [8:0]                    cin,              // 输入通道数
    input              [3:0]                    stride,           // 步幅
    input              [3:0]                    dilation,         // 膨胀率
    input              [7:0]                    left_pad,         // 左侧填充
    input              [8:0]                    input_length,     // 输入长度
    input              [11:0]                   k_total,          // 总卷积核数
    input              [9:0]                    k_group,          //归约分组
    input              [M_TAG_WIDTH-1:0]        m,               //输出时间位置
    output reg         [ADDR_WIDTH-1:0]         addr0,
    output reg         [ADDR_WIDTH-1:0]         addr1,
    output reg         [ADDR_WIDTH-1:0]         addr2,
    output reg         [ADDR_WIDTH-1:0]         addr3,
    output reg         [3:0]                    valid_mask,
    output reg                                  cin_valid
);

integer lane;
reg [3:0] cin_shift;
reg [11:0] r;
reg [11:0] kernel_index;
reg [8:0] channel_index;
reg signed [20:0] time_index;                    // 21 位有符号，保证左 Padding 时可以表示负地址。
reg [ADDR_WIDTH-1:0] lane_addr;

// 组合逻辑：根据当前输入计算cin_valid、cin_shift、addr0、addr1、addr2、addr3；本逻辑块不保存跨周期状态。
always @(*) begin

    // 把合法 Cin 映射成 log2(Cin)。
    cin_valid = 1'b1;
    case (cin)
        9'd1:   cin_shift = 4'd0;
        9'd8:   cin_shift = 4'd3;
        9'd16:  cin_shift = 4'd4;
        9'd32:  cin_shift = 4'd5;
        9'd64:  cin_shift = 4'd6;
        9'd128: cin_shift = 4'd7;
        9'd256: cin_shift = 4'd8;
        default: begin
            cin_shift = 4'd0;
            cin_valid = 1'b0;
        end
    endcase
    addr0 = {ADDR_WIDTH{1'b0}};
    addr1 = {ADDR_WIDTH{1'b0}};
    addr2 = {ADDR_WIDTH{1'b0}};
    addr3 = {ADDR_WIDTH{1'b0}};
    valid_mask = 4'b0000;
    r = 12'd0;
    kernel_index = 12'd0;
    channel_index = 9'd0;
    time_index = 21'sd0;
    lane_addr = {ADDR_WIDTH{1'b0}};
    for (lane = 0; lane < 4; lane = lane + 1) begin

        // 当前 lane 在 Cin*kernel 展平维度中的绝对位置。
        r = ({2'b00, k_group} << 2) + lane;

        // Cin 是 2 的幂：除法使用移位，取模使用 Cin-1 掩码。
        kernel_index = r >> cin_shift;
        channel_index = r[8:0] & (cin - 9'd1);

        // 使用足够宽的有符号中间量，保证左 Padding 时可以表示负地址。
        time_index = $signed({12'd0, m}) * $signed({17'd0, stride})
        - $signed({12'd0, left_pad})
        + $signed({9'd0, kernel_index}) * $signed({17'd0, dilation});
        // INT16激活按Byte编址，一个(time,channel)元素占2 Byte。
        lane_addr = input_base +
            ((time_index * $signed({12'd0, cin}) + channel_index) <<< 1);
        if (cin_valid && (r < k_total) && (time_index >= 0)
            && (time_index < $signed({1'b0, input_length}))) begin

            // 只有 K 索引和时间索引均合法时，才向 SRAM 发出该 lane 地址。
            valid_mask[lane] = 1'b1;
            case (lane)
                0: addr0 = lane_addr;
                1: addr1 = lane_addr;
                2: addr2 = lane_addr;
                3: addr3 = lane_addr;
            endcase
        end
    end
end

endmodule
