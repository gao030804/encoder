// ============================================================================
// 中文阅读导引（当前实现）
// 层描述表：索引选择Cin/Cout、K、stride、dilation、padding、输入输出长度及权重地址。
// 地址以Byte为单位，weight_block_count以32 Byte块为单位，两者不可混用。
// 本表保存旧Encoder配置，尚未迁移到最新DSCNN/LowRank网络；不要据此认为新模型已能完整运行。
// ============================================================================

//=============================================================================
// 当前 revision-3 SoundStream Encoder：63 个物理 Conv1d 的可综合配置 ROM。
// 权重布局统一为 [output_group][k_group][4][8]，每块 32 Byte。
// Depthwise 的 reduction 维是 K；Dense/Pointwise 的 reduction 维是 Cin*K。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 组合配置ROM。输入physical layer编号，输出卷积尺寸、层类型、激活类型、残差标记和权重范围。
// - Dense的k_total=Cin*K；Depthwise的k_total=K。n_groups=ceil(Cout/8)，k_groups=ceil(k_total/4)。
// - weight_flash_base为Byte地址，weight_block_count以32-Byte/256-bit块为单位。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：63层Encoder配置ROM。
// 关键变量/接口：给出每层形状、卷积类型、残差标志和权重范围；Dense的k_total=Cin×K，Depthwise为K。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module soundstream_encoder_layer_rom (
    input              [5:0]                    layer_index,
    output reg                                  valid,
    output reg         [5:0]                    logical_layer,
    output reg         [8:0]                    cin,
    output reg         [8:0]                    cout,
    output reg         [4:0]                    kernel,
    output reg         [3:0]                    stride,
    output reg         [3:0]                    dilation,
    output reg         [7:0]                    left_pad,
    output reg         [8:0]                    input_length,
    output reg         [8:0]                    output_length,
    output             [11:0]                   k_total,
    output             [9:0]                    k_groups,
    output             [5:0]                    n_groups,
    output reg                                  is_depthwise,
    output reg                                  relu_enable,
    output reg                                  elu_enable,
    output reg                                  residual_capture,
    output reg                                  residual_add,
    output reg         [23:0]                   weight_flash_base,
    output reg         [15:0]                   weight_block_count
);

integer i;
reg [23:0] base_sum;

// reduction_count决定一组输出需要多少个k_group：
// Dense：Cin*K；Depthwise：每个输出通道只对应自己的K个权重，因此为K。
wire [13:0] reduction_count = is_depthwise ? {9'd0,kernel} : cin * kernel;
// 连续赋值：组合生成k_total及其相邻接口信号，表达握手、选择或地址关系。
assign k_total  = reduction_count[11:0];
assign k_groups = (reduction_count + 14'd3) >> 2;
// 连续赋值：组合生成n_groups及其相邻接口信号，表达握手、选择或地址关系。
assign n_groups = (cout + 9'd7) >> 3;

// 63 层的 256-bit 块数；总和为 5084，即 162688 Byte。

// 辅助过程layer_blocks：封装重复计算或测试激励，便于独立检查输入、输出及边界条件。
function [15:0] layer_blocks;
    input              [5:0]                    x;
    begin
        case (x)
            0: layer_blocks=4;
            1,3,5: layer_blocks=56;
            2,4,6: layer_blocks=8;
            7: layer_blocks=64;
            8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23: layer_blocks=8;
            24: layer_blocks=16;
            25: layer_blocks=32;
            26,31,36: layer_blocks=16;
            27,28,29,30,32,33,34,35,37,38,39,40: layer_blocks=32;
            41: layer_blocks=24;
            42: layer_blocks=64;
            43: layer_blocks=128;
            44,49,54: layer_blocks=32;
            45,46,47,48,50,51,52,53,55,56,57,58: layer_blocks=128;
            59: layer_blocks=64;
            60: layer_blocks=256;
            61: layer_blocks=512;
            62: layer_blocks=1536;
            default: layer_blocks=0;
        endcase
    end
endfunction

// 组合逻辑：根据当前输入计算valid、logical_layer、cin、cout、kernel、stride；本逻辑块不保存跨周期状态。
always @(*) begin
    valid=1;
    logical_layer=layer_index;
    cin=0;
    cout=0;
    kernel=1;
    stride=1;
    dilation=1;
    left_pad=0;
    input_length=0;
    output_length=0;
    is_depthwise=0;
    relu_enable=0;
    elu_enable=0;
    residual_capture=0;
    residual_add=0;
    weight_block_count=layer_blocks(layer_index);

    // 权重在Flash/Global SRAM中逐层紧密排列。当前层base等于之前所有层
    // 256-bit块数之和乘32 Byte，从而避免维护易出错的手写十六进制地址表。
    base_sum=0;
    for (i=0; i<63; i=i+1)
        if (i < layer_index) base_sum=base_sum+({8'd0,layer_blocks(i[5:0])}<<5);
    weight_flash_base=base_sum;
    case (layer_index)

        // Stem 与 Block1：普通稠密卷积。
        0: begin
            cin=1;
            cout=16;
            kernel=7;
            left_pad=6;
            input_length=320;
            output_length=320;
        end
        1: begin
            cin=16;
            cout=16;
            kernel=7;
            left_pad=6;
            input_length=320;
            output_length=320;
            relu_enable=1;
            residual_capture=1;
        end
        2: begin
            cin=16;
            cout=16;
            input_length=320;
            output_length=320;
            relu_enable=1;
            residual_add=1;
        end
        3: begin
            cin=16;
            cout=16;
            kernel=7;
            dilation=3;
            left_pad=18;
            input_length=320;
            output_length=320;
            relu_enable=1;
            residual_capture=1;
        end
        4: begin
            cin=16;
            cout=16;
            input_length=320;
            output_length=320;
            relu_enable=1;
            residual_add=1;
        end
        5: begin
            cin=16;
            cout=16;
            kernel=7;
            dilation=9;
            left_pad=54;
            input_length=320;
            output_length=320;
            relu_enable=1;
            residual_capture=1;
        end
        6: begin
            cin=16;
            cout=16;
            input_length=320;
            output_length=320;
            relu_enable=1;
            residual_add=1;
        end
        7: begin
            cin=16;
            cout=32;
            kernel=4;
            stride=2;
            left_pad=2;
            input_length=320;
            output_length=160;
        end

        // Block2：DW + 两组 (32->8->32)，Down 为 DW + (32->16->64)。
        8,13,18: begin
            cin=32;
            cout=32;
            kernel=7;
            input_length=160;
            output_length=160;
            is_depthwise=1;
            residual_capture=1;
            if(layer_index==13)begin
                dilation=3;
                left_pad=18;
            end else if(layer_index==18)begin
                dilation=9;
                left_pad=54;
            end else left_pad=6;
        end
        9,11,14,16,19,21: begin
            cin=32;
            cout=8;
            input_length=160;
            output_length=160;
        end
        10,15,20: begin
            cin=8;
            cout=32;
            input_length=160;
            output_length=160;
            relu_enable=1;
        end
        12,17,22: begin
            cin=8;
            cout=32;
            input_length=160;
            output_length=160;
            relu_enable=1;
            residual_add=1;
        end
        23: begin
            cin=32;
            cout=32;
            kernel=8;
            stride=4;
            left_pad=4;
            input_length=160;
            output_length=40;
            is_depthwise=1;
        end
        24: begin
            cin=32;
            cout=16;
            input_length=40;
            output_length=40;
        end
        25: begin
            cin=16;
            cout=64;
            input_length=40;
            output_length=40;
        end

        // Block3：DW + 两组 (64->16->64)，Down 为 DW + (64->32->128)。
        26,31,36: begin
            cin=64;
            cout=64;
            kernel=7;
            input_length=40;
            output_length=40;
            is_depthwise=1;
            residual_capture=1;
            if(layer_index==31)begin
                dilation=3;
                left_pad=18;
            end else if(layer_index==36)begin
                dilation=9;
                left_pad=54;
            end else left_pad=6;
        end
        27,29,32,34,37,39: begin
            cin=64;
            cout=16;
            input_length=40;
            output_length=40;
        end
        28,33,38: begin
            cin=16;
            cout=64;
            input_length=40;
            output_length=40;
            relu_enable=1;
        end
        30,35,40: begin
            cin=16;
            cout=64;
            input_length=40;
            output_length=40;
            relu_enable=1;
            residual_add=1;
        end
        41: begin
            cin=64;
            cout=64;
            kernel=10;
            stride=5;
            left_pad=5;
            input_length=40;
            output_length=8;
            is_depthwise=1;
        end
        42: begin
            cin=64;
            cout=32;
            input_length=8;
            output_length=8;
        end
        43: begin
            cin=32;
            cout=128;
            input_length=8;
            output_length=8;
        end

        // Block4：DW + 两组 (128->32->128)，Down 为 DW + (128->64->256)。
        44,49,54: begin
            cin=128;
            cout=128;
            kernel=7;
            input_length=8;
            output_length=8;
            is_depthwise=1;
            residual_capture=1;
            if(layer_index==49)begin
                dilation=3;
                left_pad=18;
            end else if(layer_index==54)begin
                dilation=9;
                left_pad=54;
            end else left_pad=6;
        end
        45,47,50,52,55,57: begin
            cin=128;
            cout=32;
            input_length=8;
            output_length=8;
        end
        46,51,56: begin
            cin=32;
            cout=128;
            input_length=8;
            output_length=8;
            relu_enable=1;
        end
        48,53,58: begin
            cin=32;
            cout=128;
            input_length=8;
            output_length=8;
            relu_enable=1;
            residual_add=1;
        end
        59: begin
            cin=128;
            cout=128;
            kernel=16;
            stride=8;
            left_pad=8;
            input_length=8;
            output_length=1;
            is_depthwise=1;
        end
        60: begin
            cin=128;
            cout=64;
            input_length=1;
            output_length=1;
        end
        61: begin
            cin=64;
            cout=256;
            input_length=1;
            output_length=1;
        end
        62: begin
            cin=256;
            cout=64;
            kernel=3;
            left_pad=2;
            input_length=1;
            output_length=1;
        end
        default: begin
            valid=0;
            logical_layer=0;
            weight_block_count=0;
            weight_flash_base=0;
        end
    endcase
end

endmodule
