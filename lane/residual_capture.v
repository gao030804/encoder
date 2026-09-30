// ============================================================================
// 中文阅读导引（当前实现）
// 残差入口复制当前激活到Scratchpad，保存跨越主支路多层运算的identity数据。
// 读SRAM响应到达后再写Scratchpad；done应结合内部状态判断，不能用最后读请求发出代替完成。
// ============================================================================
//=============================================================================
// 模块：residual_capture
// 功能：在Residual Unit开始前，将当前Activation按8 Byte/word复制到Scratch。
// Activation端每次读取4个Byte，因此每个Scratch word最多需要两次读取。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - Residual Unit入口把identity张量从Activation SRAM顺序复制到Scratchpad。
// - 读响应先缓冲再组成64-bit写事务；done在最后一笔Scratch写握手完成后产生。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：残差Identity捕获单元。
// 关键变量/接口：从Activation SRAM顺序读出identity并写入Scratchpad；读响应与写请求均支持反压。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module residual_capture #(
    parameter ADDR_WIDTH = 32,
    parameter SCRATCH_ADDR_WIDTH = 11,
    parameter MAX_BYTES = 16384
) (
    input                                       clk,
    input                                       rst_n,
    input                                       start,
    output                                      start_ready,
    input              [ADDR_WIDTH-1:0]         input_base,
    input              [13:0]                   byte_count,
    output                                      busy,
    output reg                                  done,
    output reg                                  error,
    output                                      act_req_valid,
    input                                       act_req_ready,
    output             [ADDR_WIDTH-1:0]         act_addr0,
    output             [ADDR_WIDTH-1:0]         act_addr1,
    output             [ADDR_WIDTH-1:0]         act_addr2,
    output             [ADDR_WIDTH-1:0]         act_addr3,
    output             [3:0]                    act_mask,
    input                                       act_rsp_valid,
    output                                      act_rsp_ready,
    input              [63:0]                   act_rsp_data,
    output                                      scratch_wr_valid,
    input                                       scratch_wr_ready,
    output             [SCRATCH_ADDR_WIDTH-1:0] scratch_wr_addr,
    output             [63:0]                   scratch_wr_data,
    output             [7:0]                    scratch_wr_strb
);

localparam [2:0] S_IDLE=0, S_REQ_LO=1, S_WAIT_LO=2,
S_REQ_HI=3, S_WAIT_HI=4, S_WRITE=5;
reg [2:0] state;
reg [ADDR_WIDTH-1:0] base_q;
reg [13:0] bytes_q;
reg [SCRATCH_ADDR_WIDTH-1:0] word_index;
reg [63:0] low_q;
wire [14:0] word_byte_offset = {word_index,3'b000};
wire [14:0] low_base_index = word_byte_offset;
wire [14:0] high_base_index = word_byte_offset + 15'd4;
wire use_high = high_base_index < bytes_q;
// 连续赋值：组合生成start_ready及其相邻接口信号，表达握手、选择或地址关系。
assign start_ready = (state == S_IDLE);
assign busy = (state != S_IDLE);
// 连续赋值：组合生成act_req_valid及其相邻接口信号，表达握手、选择或地址关系。
assign act_req_valid = (state == S_REQ_LO) || (state == S_REQ_HI);
assign act_rsp_ready = (state == S_WAIT_LO) || (state == S_WAIT_HI);
wire request_high = (state == S_REQ_HI);
wire [14:0] request_offset = request_high ? high_base_index : low_base_index;
wire [ADDR_WIDTH-1:0] request_addr = base_q + request_offset;
// 连续赋值：组合生成act_addr0及其相邻接口信号，表达握手、选择或地址关系。
assign act_addr0 = request_addr;
assign act_addr1 = request_addr + 2'd2;
// 连续赋值：组合生成act_addr2及其相邻接口信号，表达握手、选择或地址关系。
assign act_addr2 = request_addr + 3'd4;
assign act_addr3 = request_addr + 3'd6;
// 连续赋值：组合生成act_mask及其相邻接口信号，表达握手、选择或地址关系。
assign act_mask[0] = (request_offset     < bytes_q);
assign act_mask[1] = ((request_offset+2) < bytes_q);
// 连续赋值：组合生成act_mask及其相邻接口信号，表达握手、选择或地址关系。
assign act_mask[2] = ((request_offset+4) < bytes_q);
assign act_mask[3] = ((request_offset+6) < bytes_q);
// 连续赋值：组合生成scratch_wr_valid及其相邻接口信号，表达握手、选择或地址关系。
assign scratch_wr_valid = (state == S_WRITE);
assign scratch_wr_addr  = word_index;
// 连续赋值：组合生成scratch_wr_data及其相邻接口信号，表达握手、选择或地址关系。
assign scratch_wr_data  = low_q;
assign scratch_wr_strb[0] = (word_byte_offset     < bytes_q);
// 连续赋值：组合生成scratch_wr_strb及其相邻接口信号，表达握手、选择或地址关系。
assign scratch_wr_strb[1] = ((word_byte_offset+1) < bytes_q);
assign scratch_wr_strb[2] = ((word_byte_offset+2) < bytes_q);
// 连续赋值：组合生成scratch_wr_strb及其相邻接口信号，表达握手、选择或地址关系。
assign scratch_wr_strb[3] = ((word_byte_offset+3) < bytes_q);
assign scratch_wr_strb[4] = ((word_byte_offset+4) < bytes_q);
// 连续赋值：组合生成scratch_wr_strb及其相邻接口信号，表达握手、选择或地址关系。
assign scratch_wr_strb[5] = ((word_byte_offset+5) < bytes_q);
assign scratch_wr_strb[6] = ((word_byte_offset+6) < bytes_q);
// 连续赋值：组合生成scratch_wr_strb及其相邻接口信号，表达握手、选择或地址关系。
assign scratch_wr_strb[7] = ((word_byte_offset+7) < bytes_q);
wire [14:0] next_word_offset = word_byte_offset + 15'd8;

// 时序逻辑：在时钟沿更新state、base_q、bytes_q、word_index、low_q、high_q；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state      <= S_IDLE;
        base_q     <= {ADDR_WIDTH{1'b0}};
        bytes_q    <= 14'd0;
        word_index <= {SCRATCH_ADDR_WIDTH{1'b0}};
        low_q      <= 64'd0;
        done       <= 1'b0;
        error      <= 1'b0;
    end else begin
        done <= 1'b0;
        case (state)
            S_IDLE: begin
                if (start && start_ready) begin
                    if ((byte_count == 0) || (byte_count > MAX_BYTES)) begin
                        error <= 1'b1;
                        done  <= 1'b1;
                    end else begin
                        base_q     <= input_base;
                        bytes_q    <= byte_count;
                        word_index <= 0;
                        low_q      <= 0;
                        state      <= S_REQ_LO;
                    end
                end
            end
            S_REQ_LO:
                if (act_req_valid && act_req_ready)
                state <= S_WAIT_LO;
            S_WAIT_LO: begin
                if (act_rsp_valid && act_rsp_ready) begin
                    low_q  <= act_rsp_data;
                    state  <= S_WRITE;
                end
            end
            S_REQ_HI:
                if (act_req_valid && act_req_ready)
                state <= S_WAIT_HI;
            S_WAIT_HI: begin
                if (act_rsp_valid && act_rsp_ready) begin
                    low_q <= act_rsp_data;
                    state  <= S_WRITE;
                end
            end
            S_WRITE: begin
                if (scratch_wr_valid && scratch_wr_ready) begin
                    if (next_word_offset >= bytes_q) begin
                        done  <= 1'b1;
                        state <= S_IDLE;
                    end else begin
                        word_index <= word_index + 1'b1;
                        state <= S_REQ_LO;
                    end
                end
            end
            default: begin
                state<=S_IDLE;
                error<=1'b1;
            end
        endcase
    end
end

endmodule
