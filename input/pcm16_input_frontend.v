// ============================================================================
// 中文阅读导引（当前实现）
// 接收PCM并通过量化器产生INT8样本，按写回协议存入Activation Buffer。
// 前端就绪由后续量化/写缓存决定；送满320个样本后才能启动对应单帧Encoder计算。
// ============================================================================
//=============================================================================
// 模块：pcm16_input_frontend
// 功能：接收 PCM16 流，量化为 INT8，并每 8 个样本打包成一次 64-bit 写回。
//       最后一包不足 8 字节时通过 wr_strb 标识有效字节。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 将逐采样PCM valid/ready输入转换为64-bit Activation SRAM写事务，每8个INT8样本写一次。
// - 最后不足8个样本时由write strobe标出有效字节；done表示最后一次写事务已经真正握手。
// - sample_count和写地址仅在输入或输出握手成功时更新，反压不会丢失PCM样本。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：PCM帧输入与Activation写入前端。
// 关键变量/接口：每8个PCM量化结果组成64-bit写入；末组用wr_strb标记有效byte。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module pcm16_input_frontend #(
    parameter ADDR_WIDTH = 32,
    parameter COUNT_WIDTH = 16
) (
    input                                       clk,
    input                                       rst_n,
    input                                       start,
    input              [ADDR_WIDTH-1:0]         cfg_output_base,
    input              [COUNT_WIDTH-1:0]        cfg_sample_count,
    input       signed [31:0]                   cfg_multiplier,
    input              [5:0]                    cfg_shift,
    input       signed [7:0]                    cfg_zero_point,
    output reg                                  busy,
    output reg                                  done,
    input                                       pcm_valid,
    output                                      pcm_ready,
    input       signed [15:0]                   pcm_data,
    output reg                                  wr_valid,
    input                                       wr_ready,
    output reg         [ADDR_WIDTH-1:0]         wr_addr,
    output reg         [63:0]                   wr_data,
    output reg         [7:0]                    wr_strb
);

reg [COUNT_WIDTH-1:0] accepted_count;
reg [COUNT_WIDTH-1:0] sample_count_q;
reg [ADDR_WIDTH-1:0]  next_wr_addr;
reg [2:0]             pack_count;
reg [63:0]            pack_data;
reg                    wr_last;
reg signed [31:0]      multiplier_q;
reg [5:0]              shift_q;
reg signed [7:0]       zero_point_q;
wire q_in_valid;
wire q_in_ready;
wire q_in_last;
wire q_out_valid;
wire q_out_ready;
wire signed [15:0] q_out_data;
wire q_out_last;
reg [63:0] pack_next;
// 连续赋值：组合生成q_in_valid及其相邻接口信号，表达握手、选择或地址关系。
assign q_in_valid = busy && pcm_valid &&
    (accepted_count < sample_count_q);
// 连续赋值：组合生成pcm_ready及其相邻接口信号，表达握手、选择或地址关系。
assign pcm_ready  = busy && q_in_ready &&
    (accepted_count < sample_count_q);
// 连续赋值：组合生成q_in_last及其相邻接口信号，表达握手、选择或地址关系。
assign q_in_last  = (accepted_count == (sample_count_q - 1'b1));

// 写口发生握手的同一周期可以继续接收下一字节，不额外插入气泡。
// 连续赋值：组合生成q_out_ready及其相邻接口信号，表达握手、选择或地址关系。
assign q_out_ready = busy && (!wr_valid || wr_ready);

// 组合逻辑：根据当前输入计算pack_next；本逻辑块不保存跨周期状态。
always @(*) begin
    pack_next = pack_data;
    pack_next[16*pack_count +: 16] = q_out_data;
end

pcm16_to_int8_quantizer u_quantizer (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .in_valid                    (q_in_valid),
    .in_ready                    (q_in_ready),
    .in_pcm                      (pcm_data),
    .in_last                     (q_in_last),
    .cfg_multiplier              (multiplier_q),
    .cfg_shift                   (shift_q),
    .cfg_zero_point              (zero_point_q),
    .out_valid                   (q_out_valid),
    .out_ready                   (q_out_ready),
    .out_data                    (q_out_data),
    .out_last                    (q_out_last)
);

// 时序逻辑：在时钟沿更新busy、done、accepted_count、sample_count_q、next_wr_addr、pack_count；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        busy           <= 1'b0;
        done           <= 1'b0;
        accepted_count <= {COUNT_WIDTH{1'b0}};
        sample_count_q  <= {COUNT_WIDTH{1'b0}};
        next_wr_addr   <= {ADDR_WIDTH{1'b0}};
        pack_count     <= 3'd0;
        pack_data      <= 64'd0;
        wr_valid       <= 1'b0;
        wr_addr        <= {ADDR_WIDTH{1'b0}};
        wr_data        <= 64'd0;
        wr_strb        <= 8'd0;
        wr_last        <= 1'b0;
        multiplier_q   <= 32'sd0;
        shift_q        <= 6'd0;
        zero_point_q   <= 8'sd0;
    end else begin
        done <= 1'b0;
        if (!busy && start) begin
            accepted_count <= {COUNT_WIDTH{1'b0}};
            sample_count_q  <= cfg_sample_count;
            next_wr_addr   <= cfg_output_base;
            pack_count     <= 3'd0;
            pack_data      <= 64'd0;
            wr_valid       <= 1'b0;
            wr_last        <= 1'b0;
            multiplier_q   <= cfg_multiplier;
            shift_q        <= cfg_shift;
            zero_point_q   <= cfg_zero_point;
            if (cfg_sample_count == 0) begin
                busy <= 1'b0;
                done <= 1'b1;
            end else begin
                busy <= 1'b1;
            end
        end else begin
            if (q_in_valid && q_in_ready) begin
                accepted_count <= accepted_count + 1'b1;
            end
            if (wr_valid && wr_ready) begin
                wr_valid <= 1'b0;
                if (wr_last) begin
                    busy    <= 1'b0;
                    done    <= 1'b1;
                    wr_last <= 1'b0;
                end
            end
            if (q_out_valid && q_out_ready) begin
                if ((pack_count == 3'd3) || q_out_last) begin
                    wr_valid     <= 1'b1;
                    wr_addr      <= next_wr_addr;
                    wr_data      <= pack_next;
                    case (pack_count)
                        3'd0: wr_strb <= 8'h03;
                        3'd1: wr_strb <= 8'h0f;
                        3'd2: wr_strb <= 8'h3f;
                        default: wr_strb <= 8'hff;
                    endcase
                    wr_last      <= q_out_last;
                    next_wr_addr <= next_wr_addr + 8;
                    pack_count   <= 3'd0;
                    pack_data    <= 64'd0;
                end else begin
                    pack_data  <= pack_next;
                    pack_count <= pack_count + 1'b1;
                end
            end
        end
    end
end

endmodule
