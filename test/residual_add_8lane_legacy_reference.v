// Frozen test-only reference: exact pre-optimization RTL, excluded from synthesis.
// ============================================================================
// 中文阅读导引（当前实现）
// 主支路INT8结果与Scratchpad中的identity逐通道相加、饱和并写回。
// 两条支路必须具有相同量化Scale，本模块不执行独立的支路Scale转换。
// 输出反压会延缓完成，帧级调度器须等待最后一次实际写回。
// ============================================================================
//=============================================================================
// 模块：residual_add_8lane
// 功能：将主支路8路INT8与Residual Scratch中的8路INT8执行饱和加法。
//
// 量化约束：两路数据必须已经位于相同的signed-INT8 scale，zero_point=0。
// 该约束对应训练端HardwareResidualAdd的shared-scale量化方式。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 根据主分支输出Byte地址换算Scratch word地址，读出8路identity并逐lane相加。
// - 加法使用signed 9-bit中间值并饱和到[-128,127]；write strobe为0的lane不产生有效结果。
// - 当前要求主分支与identity使用相同Scale；非单位Residual Scale需额外Requant参数。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：八路残差加法器。
// 关键变量/接口：main和identity按lane作signed 9-bit加法并饱和到INT8；要求两分支量化Scale一致。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module residual_add_8lane_legacy_reference #(
    parameter ADDR_WIDTH = 32,
    parameter SCRATCH_ADDR_WIDTH = 11,
    parameter SCALE_ALIGN_ENABLE = 0
) (
    input                                       clk,
    input                                       rst_n,
    input                                       in_valid,
    output                                      in_ready,
    output                                      busy,
    input              [ADDR_WIDTH-1:0]         in_addr,
    input              [ADDR_WIDTH-1:0]         output_base,
    input              [63:0]                   in_data,
    input              [7:0]                    in_strb,
    // 三组{UINT8 shift,INT32 multiplier}：identity、main、residual_scale。
    // 参数在in_valid&&in_ready时锁存，防止SRAM响应等待期间参数变化。
    input              [119:0]                  scale_params,
    output                                      scratch_rd_valid,
    input                                       scratch_rd_ready,
    output             [SCRATCH_ADDR_WIDTH-1:0] scratch_rd_addr,
    input                                       scratch_rsp_valid,
    input              [63:0]                   scratch_rsp_data,
    output reg                                  out_valid,
    input                                       out_ready,
    output reg         [ADDR_WIDTH-1:0]         out_addr,
    output reg         [63:0]                   out_data,
    output reg         [7:0]                    out_strb,
    output reg                                  error
);

localparam [1:0] S_IDLE = 2'd0, S_READ = 2'd1, S_WAIT = 2'd2;
reg [1:0] state;
reg [ADDR_WIDTH-1:0] addr_q;
reg [ADDR_WIDTH-1:0] base_q;
reg [63:0] data_q;
reg [7:0] strb_q;
reg [119:0] params_q;
wire [ADDR_WIDTH-1:0] byte_offset = addr_q - base_q;
wire [ADDR_WIDTH-1:0] input_byte_offset = in_addr - output_base;
// 连续赋值：组合生成in_ready及其相邻接口信号，表达握手、选择或地址关系。
assign in_ready = (state == S_IDLE) && (!out_valid || out_ready);

// state回到IDLE但out_valid仍被反压时，模块仍未排空。
// 连续赋值：组合生成busy及其相邻接口信号，表达握手、选择或地址关系。
assign busy = (state != S_IDLE) || out_valid;
assign scratch_rd_valid = (state == S_READ);
// 连续赋值：组合生成scratch_rd_addr及其相邻接口信号，表达握手、选择或地址关系。
assign scratch_rd_addr = byte_offset[SCRATCH_ADDR_WIDTH+2:3];
integer lane;
reg signed [63:0] lane_sum;
reg signed [15:0] identity_aligned, main_aligned;
reg [63:0] added_data;

// 16×32乘积最多48位，64位容器容纳绝对值、舍入偏置和结果。
// 负数先对绝对值舍入再恢复符号，负半整数远离零。
function signed [63:0] requant16;
    input signed [15:0] value;
    input signed [31:0] multiplier;
    input [7:0] shift;
    reg signed [63:0] product, mag;
    begin
        product = value * multiplier;
        if (shift == 0) requant16 = product;
        else if (shift >= 64) requant16 = 64'sd0;
        else begin
            mag = (product < 0) ? -product : product;
            mag = (mag + (64'sd1 << (shift-1))) >>> shift;
            requant16 = (product < 0) ? -mag : mag;
        end
    end
endfunction

function signed [15:0] saturate16;
    input signed [63:0] value;
    begin
        if (value > 64'sd32767) saturate16 = 16'sh7fff;
        else if (value < -64'sd32768) saturate16 = 16'sh8000;
        else saturate16 = value[15:0];
    end
endfunction

// 组合逻辑：根据当前输入计算added_data、lane_sum、lane；本逻辑块不保存跨周期状态。
always @(*) begin
    added_data = 64'd0;
    lane_sum = 17'sd0;
    identity_aligned = 0;
    main_aligned = 0;
    for (lane=0; lane<4; lane=lane+1) begin
        if (SCALE_ALIGN_ENABLE) begin
            identity_aligned = saturate16(requant16(scratch_rsp_data[16*lane +: 16], params_q[31:0], params_q[39:32]));
            main_aligned = saturate16(requant16(data_q[16*lane +: 16], params_q[71:40], params_q[79:72]));
            lane_sum = $signed(identity_aligned) + requant16(main_aligned, params_q[111:80], params_q[119:112]);
        end else begin
            lane_sum = $signed(data_q[16*lane +: 16]) +
                $signed(scratch_rsp_data[16*lane +: 16]);
        end
        if (!(strb_q[2*lane] || strb_q[2*lane+1]))
            added_data[16*lane +: 16] = 16'd0;
        else if (lane_sum > 17'sd32767)
            added_data[16*lane +: 16] = 16'h7fff;
        else if (lane_sum < -17'sd32768)
            added_data[16*lane +: 16] = 16'h8000;
        else
        added_data[16*lane +: 16] = lane_sum[15:0];
    end
end

// 时序逻辑：在时钟沿更新state、addr_q、base_q、data_q、strb_q、out_valid；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state     <= S_IDLE;
        addr_q    <= {ADDR_WIDTH{1'b0}};
        base_q    <= {ADDR_WIDTH{1'b0}};
        data_q    <= 64'd0;
        strb_q    <= 8'd0;
        params_q  <= 120'd0;
        out_valid <= 1'b0;
        out_addr  <= {ADDR_WIDTH{1'b0}};
        out_data  <= 64'd0;
        out_strb  <= 8'd0;
        error     <= 1'b0;
    end else begin
        if (out_valid && out_ready)
            out_valid <= 1'b0;
        case (state)
            S_IDLE: begin
                if (in_valid && in_ready) begin
                    addr_q <= in_addr;
                    base_q <= output_base;
                    data_q <= in_data;
                    strb_q <= in_strb;
                    params_q <= scale_params;
                    if ((in_addr < output_base) ||
                        (input_byte_offset[2:0] != 3'b000)) begin
                        error <= 1'b1;
                    end
                    state <= S_READ;
                end
            end
            S_READ:
                if (scratch_rd_valid && scratch_rd_ready)
                state <= S_WAIT;
            S_WAIT: begin
                if (scratch_rsp_valid) begin
                    out_valid <= 1'b1;
                    out_addr  <= addr_q;
                    out_data  <= added_data;
                    out_strb  <= strb_q;
                    state     <= S_IDLE;
                end
            end
            default: begin
                state <= S_IDLE;
                error <= 1'b1;
            end
        endcase
    end
end

endmodule
