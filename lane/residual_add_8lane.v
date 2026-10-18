// ============================================================================
// W8A16 Residual Add：保留历史8lane模块名，64-bit写拍实际包含4路signed INT16。
// SCALE_ALIGN_ENABLE=1：
//   I = sat16(requant(identity,m1,s1))
//   M = sat16(requant(main,m2,s2))
//   Y = sat16(I + requant(M,m3,s3))
// 第二次requant只限幅到[-65535,65535]，绝不能提前sat16，否则相消结果会改变。
// 48-bit乘积/绝对值 + Guard舍入 + 饱和感知窄位结果，最后用signed18相加。
// SCALE_ALIGN_ENABLE=0：两路INT16直接饱和相加，要求Scale已对齐。
// 四状态流水：S_WAIT锁存第一段结果，S_WAIT2完成第二次量化/相加并输出。
// 接口和算术结果不变；相对原本地三状态版，每笔事务增加一拍处理延迟。
// ============================================================================
module residual_add_8lane #(
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

localparam [2:0] S_IDLE = 3'd0, S_READ = 3'd1,
                 S_WAIT = 3'd2, S_WAIT2 = 3'd3;
reg [2:0] state;
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
integer lane, align_lane, reset_lane;
reg signed [17:0] lane_sum;
// 每个64-bit写拍含4个INT16，_w为第一段组合结果，_q为跨拍保持的数据。
// Scale关闭时也锁存原始两支路，不能在S_WAIT2继续使用可能已变化的SRAM数据。
reg signed [15:0] identity_aligned_w [0:3];
reg signed [15:0] main_aligned_w [0:3];
reg signed [15:0] identity_aligned_q [0:3];
reg signed [15:0] main_aligned_q [0:3];
reg signed [16:0] main_requant_clip;
reg [63:0] added_data;

// 第一次量化直接sat16。|INT16 x INT32|<=2^46，48位内取绝对值不会溢出。
// 绝对值上half-up只需Guard位，不需Sticky。先判饱和，再用16位进位/恢复符号。
function signed [15:0] requant_sat16;
    input signed [15:0] value;
    input signed [31:0] multiplier;
    input [7:0] shift;
    reg signed [47:0] product;
    reg [47:0] mag, q;
    reg round_up;
    reg [15:0] rounded16;
    begin
        product = $signed(value) * $signed(multiplier);
        mag = 48'd0;
        q = 48'd0;
        round_up = 1'b0;
        rounded16 = 16'd0;
        if (shift == 0) begin
            if (product > 48'sd32767) requant_sat16 = 16'sh7fff;
            else if (product < -48'sd32768) requant_sat16 = 16'sh8000;
            else requant_sat16 = product[15:0];
        end else if (shift >= 8'd48) begin
            requant_sat16 = 16'sd0;
        end else begin
            // shift=47仍必须计算：最大乘积2^46恰好为半值，结果为+1。
            mag = product[47] ? -product : product;
            q = mag >> shift;
            round_up = mag[shift-1'b1];
            if (!product[47]) begin
                if (q >= 48'd32767) requant_sat16 = 16'sh7fff;
                else begin
                    rounded16 = {1'b0,q[14:0]} + {15'd0,round_up};
                    requant_sat16 = $signed(rounded16);
                end
            end else begin
                if ((q > 48'd32767) || ((q == 48'd32767) && round_up))
                    requant_sat16 = 16'sh8000;
                else begin
                    rounded16 = {1'b0,q[14:0]} + {15'd0,round_up};
                    requant_sat16 = -$signed(rounded16);
                end
            end
        end
    end
endfunction

// 第二次量化后还要与INT16 identity相加，因此仅clip到对称17位范围。
// 超过+/-65535时无论identity取何值，最终sat16结果都不会改变。
// 不先构造48位q+1，而是在非饱和路径上只做17位增量。
function signed [16:0] requant_clip17;
    input signed [15:0] value;
    input signed [31:0] multiplier;
    input [7:0] shift;
    reg signed [47:0] product;
    reg [47:0] mag, q;
    reg round_up;
    reg [16:0] rounded17;
    begin
        product = $signed(value) * $signed(multiplier);
        mag = 48'd0;
        q = 48'd0;
        round_up = 1'b0;
        rounded17 = 17'd0;
        if (shift == 0) begin
            if (product > 48'sd65535) requant_clip17 = 17'sd65535;
            else if (product < -48'sd65535) requant_clip17 = -17'sd65535;
            else requant_clip17 = product[16:0];
        end else if (shift >= 8'd48) begin
            requant_clip17 = 17'sd0;
        end else begin
            mag = product[47] ? -product : product;
            q = mag >> shift;
            round_up = mag[shift-1'b1];
            if (!product[47]) begin
                if (q >= 48'd65535) requant_clip17 = 17'sd65535;
                else begin
                    rounded17 = {1'b0,q[15:0]} + {16'd0,round_up};
                    requant_clip17 = $signed(rounded17);
                end
            end else begin
                if ((q >= 48'd65535) || ((q == 48'd65534) && round_up))
                    requant_clip17 = -17'sd65535;
                else begin
                    rounded17 = {1'b0,q[15:0]} + {16'd0,round_up};
                    requant_clip17 = -$signed(rounded17);
                end
            end
        end
    end
endfunction

// 第一段：两支路独立做首次Scale对齐及INT16饱和，在SRAM响应有效沿锁存。
always @(*) begin
    for (align_lane=0; align_lane<4; align_lane=align_lane+1) begin
        if (SCALE_ALIGN_ENABLE) begin
            identity_aligned_w[align_lane] = requant_sat16(
                scratch_rsp_data[16*align_lane +: 16], params_q[31:0], params_q[39:32]);
            main_aligned_w[align_lane] = requant_sat16(
                data_q[16*align_lane +: 16], params_q[71:40], params_q[79:72]);
        end else begin
            identity_aligned_w[align_lane] = scratch_rsp_data[16*align_lane +: 16];
            main_aligned_w[align_lane] = data_q[16*align_lane +: 16];
        end
    end
end

// 第二段：只读_q，切断SRAM响应到两次串行requant的长组合路径。
// S_WAIT2内完成Residual Scale量化、signed18求和及最终INT16饱和。
always @(*) begin
    added_data = 64'd0;
    lane_sum = 18'sd0;
    main_requant_clip = 0;
    for (lane=0; lane<4; lane=lane+1) begin
        if (SCALE_ALIGN_ENABLE) begin
            main_requant_clip = requant_clip17(main_aligned_q[lane], params_q[111:80], params_q[119:112]);
            lane_sum = {{2{identity_aligned_q[lane][15]}},identity_aligned_q[lane]} +
                       {main_requant_clip[16],main_requant_clip};
        end else begin
            lane_sum = {{2{main_aligned_q[lane][15]}},main_aligned_q[lane]} +
                       {{2{identity_aligned_q[lane][15]}},identity_aligned_q[lane]};
        end
        if (!(strb_q[2*lane] || strb_q[2*lane+1]))
            added_data[16*lane +: 16] = 16'd0;
        else if (lane_sum > 18'sd32767)
            added_data[16*lane +: 16] = 16'h7fff;
        else if (lane_sum < -18'sd32768)
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
        for (reset_lane=0; reset_lane<4; reset_lane=reset_lane+1) begin
            identity_aligned_q[reset_lane] <= 16'sd0;
            main_aligned_q[reset_lane] <= 16'sd0;
        end
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
                    // 该沿消费SRAM响应；后续不再依赖scratch_rsp_data保持。
                    for (reset_lane=0; reset_lane<4; reset_lane=reset_lane+1) begin
                        identity_aligned_q[reset_lane] <= identity_aligned_w[reset_lane];
                        main_aligned_q[reset_lane] <= main_aligned_w[reset_lane];
                    end
                    state <= S_WAIT2;
                end
            end
            S_WAIT2: begin
                // 只停留一拍。输出进入原有valid/ready保持寄存器后回IDLE。
                // out_ready=0时out_valid及数据保持，in_ready仍为0，不能覆盖未消费结果。
                out_valid <= 1'b1;
                out_addr  <= addr_q;
                out_data  <= added_data;
                out_strb  <= strb_q;
                state     <= S_IDLE;
            end
            default: begin
                state <= S_IDLE;
                error <= 1'b1;
            end
        endcase
    end
end

endmodule
