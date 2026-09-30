// ============================================================================
// 中文阅读导引（当前实现）
// Activation SRAM组织：A/B各4个1024×16-bit Bank，各8 KiB，总16 KiB。
// byte_offset=addr-base；row=offset>>3；bank=offset[2:1]；byte_sel=offset[0]。
// 四路读地址可分别选Bank，同一Bank若要求不同row则存在端口冲突，由检测逻辑报告。
// 一次写64 bit，wr_strb逐字节控制写使能；读写角色由顶层的buffer_select决定。
// ============================================================================
//=============================================================================
// 模块：activation_buffer_pingpong
// 功能：两块可交换角色的片上激活缓冲区。
//
// 逻辑组织：每个Buffer = 4 Bank × 16 bit × ROWS，共ROWS×8字节。
// 物理实现：每个Bank实例化一个OpenTitan prim_ram_1p适配器。
//
// byte地址映射：
//   row      = (byte_addr - buffer_base) >> 3
//   bank     = (byte_addr - buffer_base)[2:1]
//   byte_sel = (byte_addr - buffer_base)[0]
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - A/B各由4个16-bit SRAM Bank组成，地址低3位映射bank和byte，高位映射row。
// - 一次64-bit写回可更新8个INT8；wr_strb逐字节控制，未使能字节保持原值。
// - 四路读地址若访问同一Bank但row不同会产生端口冲突并报告error。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：Activation SRAM Ping-Pong缓存。
// 关键变量/接口：A/B Buffer交替作为输入和输出；4个16-bit Bank支持四路byte读取和64-bit带strobe写入。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
// W8A16版本：每个Ping/Pong Buffer为4 Bank x 2048 Row x 16 bit = 16 KiB。
// 每个逻辑lane由适配器再分成4个512x16宏；四lane保持独立地址，不能合并端口。
module activation_buffer_pingpong #(
    parameter ADDR_WIDTH = 32,
    parameter ROW_WIDTH  = 11,
    parameter ROWS       = 2048
) (
    input                                       clk,
    input                                       rst_n,
    input                                       rd_buffer_select,
    input              [ADDR_WIDTH-1:0]         rd_buffer_base,
    input                                       rd_req_valid,
    output                                      rd_req_ready,
    input              [ADDR_WIDTH-1:0]         rd_addr0,
    input              [ADDR_WIDTH-1:0]         rd_addr1,
    input              [ADDR_WIDTH-1:0]         rd_addr2,
    input              [ADDR_WIDTH-1:0]         rd_addr3,
    input              [3:0]                    rd_mask,
    output                                      rd_rsp_valid,
    input                                       rd_rsp_ready,
    output             [63:0]                   rd_rsp_data,
    input                                       wr_buffer_select,
    input              [ADDR_WIDTH-1:0]         wr_buffer_base,
    input                                       wr_valid,
    output                                      wr_ready,
    input              [ADDR_WIDTH-1:0]         wr_addr,
    input              [63:0]                   wr_data,
    input              [7:0]                    wr_strb,
    output reg                                  access_error
);

localparam BUFFER_BYTES = ROWS * 8;

//-------------------------------------------------------------------------
// 四路byte地址译码。一次请求中，同一Bank只能访问同一个row。
//-------------------------------------------------------------------------
reg [ROW_WIDTH-1:0] decoded_row [0:3];
reg                 decoded_need [0:3];
reg [1:0]           lane_bank_d [0:3];
reg                 lane_byte_d [0:3];
reg                 decode_conflict;
reg                 decode_oob;
integer decode_lane;
integer decode_bank;
reg [ADDR_WIDTH-1:0] decode_addr;
reg [ADDR_WIDTH-1:0] decode_offset;

// 组合逻辑：根据当前输入计算decode_conflict、decode_oob、decode_addr、decode_offset、decode_bank、decode_lane；本逻辑块不保存跨周期状态。
always @(*) begin
    decoded_row[0]  = {ROW_WIDTH{1'b0}};
    decoded_row[1]  = {ROW_WIDTH{1'b0}};
    decoded_row[2]  = {ROW_WIDTH{1'b0}};
    decoded_row[3]  = {ROW_WIDTH{1'b0}};
    decoded_need[0] = 1'b0;
    decoded_need[1] = 1'b0;
    decoded_need[2] = 1'b0;
    decoded_need[3] = 1'b0;
    decode_conflict = 1'b0;
    decode_oob      = 1'b0;
    decode_addr     = {ADDR_WIDTH{1'b0}};
    decode_offset   = {ADDR_WIDTH{1'b0}};
    decode_bank     = 0;
    for (decode_lane = 0; decode_lane < 4;
        decode_lane = decode_lane + 1) begin
        lane_bank_d[decode_lane] = 2'd0;
        lane_byte_d[decode_lane] = 1'b0;
        case (decode_lane)
            0: decode_addr = rd_addr0;
            1: decode_addr = rd_addr1;
            2: decode_addr = rd_addr2;
            default: decode_addr = rd_addr3;
        endcase
        if (rd_mask[decode_lane]) begin
            if ((decode_addr < rd_buffer_base) ||
                ((decode_addr-rd_buffer_base) >= BUFFER_BYTES)) begin
                decode_oob = 1'b1;
            end else begin
                decode_offset = decode_addr - rd_buffer_base;
                decode_bank = decode_offset[2:1];
                lane_bank_d[decode_lane] = decode_offset[2:1];
                lane_byte_d[decode_lane] = decode_offset[0];
                if (!decoded_need[decode_bank]) begin
                    decoded_need[decode_bank] = 1'b1;
                    decoded_row[decode_bank] = decode_offset[ROW_WIDTH+2:3];
                end else if (decoded_row[decode_bank] !=
                    decode_offset[ROW_WIDTH+2:3]) begin
                    decode_conflict = 1'b1;
                end
            end
        end
    end
end

//-------------------------------------------------------------------------
// 请求仲裁与8个SRAM Bank。
// 正常Ping-Pong时读写落在不同Buffer，可在同一周期并行进行。
//-------------------------------------------------------------------------
reg rd_meta_valid;
reg rd_buffer_select_q;
reg [1:0] lane_bank_q [0:3];
reg       lane_byte_q [0:3];
reg [3:0] lane_mask_q;
integer seq_lane;
reg [63:0] rsp_fifo [0:7];
reg [2:0]  rsp_wr_ptr;
reg [2:0]  rsp_rd_ptr;
reg [3:0]  rsp_count;
wire rsp_pop = rd_rsp_valid && rd_rsp_ready;
wire [4:0] reserved_rsp_count = {1'b0, rsp_count} +
    (rd_meta_valid ? 5'd1 : 5'd0);

// 返回 FIFO 和一拍 SRAM 读流水级共同提供反压；正常情况下可每拍读一次。
// 连续赋值：组合生成rd_req_ready及其相邻接口信号，表达握手、选择或地址关系。
assign rd_req_ready = (reserved_rsp_count < 5'd8) || rsp_pop;
wire rd_fire = rd_req_valid && rd_req_ready;
wire rd_issue = rd_fire && !decode_conflict && !decode_oob;
wire same_buffer = (rd_buffer_select == wr_buffer_select);
// 连续赋值：组合生成wr_ready及其相邻接口信号，表达握手、选择或地址关系。
assign wr_ready = !same_buffer || !rd_fire;
wire [ADDR_WIDTH-1:0] wr_offset = wr_addr - wr_buffer_base;
wire [ROW_WIDTH-1:0] wr_row = wr_offset[ROW_WIDTH+2:3];
wire wr_in_range = (wr_addr >= wr_buffer_base) &&
    (wr_offset <= (BUFFER_BYTES-8));
wire wr_aligned = (wr_offset[2:0] == 3'b000);
wire wr_fire = wr_valid && wr_ready && wr_in_range && wr_aligned;
wire a_wr0 = wr_fire && !wr_buffer_select && |wr_strb[1:0];
wire a_wr1 = wr_fire && !wr_buffer_select && |wr_strb[3:2];
wire a_wr2 = wr_fire && !wr_buffer_select && |wr_strb[5:4];
wire a_wr3 = wr_fire && !wr_buffer_select && |wr_strb[7:6];
wire b_wr0 = wr_fire &&  wr_buffer_select && |wr_strb[1:0];
wire b_wr1 = wr_fire &&  wr_buffer_select && |wr_strb[3:2];
wire b_wr2 = wr_fire &&  wr_buffer_select && |wr_strb[5:4];
wire b_wr3 = wr_fire &&  wr_buffer_select && |wr_strb[7:6];
wire a_rd0 = rd_issue && !rd_buffer_select && decoded_need[0];
wire a_rd1 = rd_issue && !rd_buffer_select && decoded_need[1];
wire a_rd2 = rd_issue && !rd_buffer_select && decoded_need[2];
wire a_rd3 = rd_issue && !rd_buffer_select && decoded_need[3];
wire b_rd0 = rd_issue &&  rd_buffer_select && decoded_need[0];
wire b_rd1 = rd_issue &&  rd_buffer_select && decoded_need[1];
wire b_rd2 = rd_issue &&  rd_buffer_select && decoded_need[2];
wire b_rd3 = rd_issue &&  rd_buffer_select && decoded_need[3];
wire [15:0] a_q0,a_q1,a_q2,a_q3,b_q0,b_q1,b_q2,b_q3;
wire unused_a_v0,unused_a_v1,unused_a_v2,unused_a_v3;
wire unused_b_v0,unused_b_v1,unused_b_v2,unused_b_v3;
opentitan_sram_1p_adapter #(.WIDTH(16),
    .DEPTH                       (ROWS),
    .DATA_BITS_PER_MASK          (8)) u_a_bank0 (
    .clk_i                       (clk),
    .rst_ni                      (rst_n),
    .req_i                       (a_wr0|a_rd0),
    .write_i                     (a_wr0),
    .addr_i                      (a_wr0?wr_row:decoded_row[0]),
    .wdata_i                     (wr_data[15:0]),
    .wmask_i                     (wr_strb[1:0]),
    .rdata_o                     (a_q0),
    .rvalid_o                    (unused_a_v0));
opentitan_sram_1p_adapter #(.WIDTH(16),
    .DEPTH                       (ROWS),
    .DATA_BITS_PER_MASK          (8)) u_a_bank1 (
    .clk_i                       (clk),
    .rst_ni                      (rst_n),
    .req_i                       (a_wr1|a_rd1),
    .write_i                     (a_wr1),
    .addr_i                      (a_wr1?wr_row:decoded_row[1]),
    .wdata_i                     (wr_data[31:16]),
    .wmask_i                     (wr_strb[3:2]),
    .rdata_o                     (a_q1),
    .rvalid_o                    (unused_a_v1));
opentitan_sram_1p_adapter #(.WIDTH(16),
    .DEPTH                       (ROWS),
    .DATA_BITS_PER_MASK          (8)) u_a_bank2 (
    .clk_i                       (clk),
    .rst_ni                      (rst_n),
    .req_i                       (a_wr2|a_rd2),
    .write_i                     (a_wr2),
    .addr_i                      (a_wr2?wr_row:decoded_row[2]),
    .wdata_i                     (wr_data[47:32]),
    .wmask_i                     (wr_strb[5:4]),
    .rdata_o                     (a_q2),
    .rvalid_o                    (unused_a_v2));
opentitan_sram_1p_adapter #(.WIDTH(16),
    .DEPTH                       (ROWS),
    .DATA_BITS_PER_MASK          (8)) u_a_bank3 (
    .clk_i                       (clk),
    .rst_ni                      (rst_n),
    .req_i                       (a_wr3|a_rd3),
    .write_i                     (a_wr3),
    .addr_i                      (a_wr3?wr_row:decoded_row[3]),
    .wdata_i                     (wr_data[63:48]),
    .wmask_i                     (wr_strb[7:6]),
    .rdata_o                     (a_q3),
    .rvalid_o                    (unused_a_v3));
opentitan_sram_1p_adapter #(.WIDTH(16),
    .DEPTH                       (ROWS),
    .DATA_BITS_PER_MASK          (8)) u_b_bank0 (
    .clk_i                       (clk),
    .rst_ni                      (rst_n),
    .req_i                       (b_wr0|b_rd0),
    .write_i                     (b_wr0),
    .addr_i                      (b_wr0?wr_row:decoded_row[0]),
    .wdata_i                     (wr_data[15:0]),
    .wmask_i                     (wr_strb[1:0]),
    .rdata_o                     (b_q0),
    .rvalid_o                    (unused_b_v0));
opentitan_sram_1p_adapter #(.WIDTH(16),
    .DEPTH                       (ROWS),
    .DATA_BITS_PER_MASK          (8)) u_b_bank1 (
    .clk_i                       (clk),
    .rst_ni                      (rst_n),
    .req_i                       (b_wr1|b_rd1),
    .write_i                     (b_wr1),
    .addr_i                      (b_wr1?wr_row:decoded_row[1]),
    .wdata_i                     (wr_data[31:16]),
    .wmask_i                     (wr_strb[3:2]),
    .rdata_o                     (b_q1),
    .rvalid_o                    (unused_b_v1));
opentitan_sram_1p_adapter #(.WIDTH(16),
    .DEPTH                       (ROWS),
    .DATA_BITS_PER_MASK          (8)) u_b_bank2 (
    .clk_i                       (clk),
    .rst_ni                      (rst_n),
    .req_i                       (b_wr2|b_rd2),
    .write_i                     (b_wr2),
    .addr_i                      (b_wr2?wr_row:decoded_row[2]),
    .wdata_i                     (wr_data[47:32]),
    .wmask_i                     (wr_strb[5:4]),
    .rdata_o                     (b_q2),
    .rvalid_o                    (unused_b_v2));
opentitan_sram_1p_adapter #(.WIDTH(16),
    .DEPTH                       (ROWS),
    .DATA_BITS_PER_MASK          (8)) u_b_bank3 (
    .clk_i                       (clk),
    .rst_ni                      (rst_n),
    .req_i                       (b_wr3|b_rd3),
    .write_i                     (b_wr3),
    .addr_i                      (b_wr3?wr_row:decoded_row[3]),
    .wdata_i                     (wr_data[63:48]),
    .wmask_i                     (wr_strb[7:6]),
    .rdata_o                     (b_q3),
    .rvalid_o                    (unused_b_v3));
wire [15:0] read_q0 = rd_buffer_select_q ? b_q0 : a_q0;
wire [15:0] read_q1 = rd_buffer_select_q ? b_q1 : a_q1;
wire [15:0] read_q2 = rd_buffer_select_q ? b_q2 : a_q2;
wire [15:0] read_q3 = rd_buffer_select_q ? b_q3 : a_q3;
// 连续赋值：组合生成rd_rsp_valid及其相邻接口信号，表达握手、选择或地址关系。
assign rd_rsp_valid = (rsp_count != 0);
assign rd_rsp_data  = rsp_fifo[rsp_rd_ptr];
reg [63:0] assembled_read_data;
integer assemble_lane;

// 组合逻辑：根据当前输入计算assembled_read_data、assemble_lane、seq_lane；本逻辑块不保存跨周期状态。
always @(*) begin
    assembled_read_data = 64'd0;
    for (assemble_lane=0;assemble_lane<4;assemble_lane=assemble_lane+1) begin
        if (!lane_mask_q[assemble_lane]) begin
            assembled_read_data[16*assemble_lane +: 16] = 16'd0;
        end else begin
            case (lane_bank_q[assemble_lane])
                2'd0: assembled_read_data[16*assemble_lane +: 16] = read_q0;
                2'd1: assembled_read_data[16*assemble_lane +: 16] = read_q1;
                2'd2: assembled_read_data[16*assemble_lane +: 16] = read_q2;
                default: assembled_read_data[16*assemble_lane +: 16] = read_q3;
            endcase
        end
    end
end

// 时序逻辑：在时钟沿更新rd_meta_valid、rd_buffer_select_q、lane_mask_q、rsp_wr_ptr、rsp_rd_ptr、rsp_count；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_meta_valid <= 1'b0;
        rd_buffer_select_q <= 1'b0;
        lane_mask_q <= 4'd0;
        rsp_wr_ptr <= 3'd0;
        rsp_rd_ptr <= 3'd0;
        rsp_count <= 4'd0;
        access_error <= 1'b0;
        for (seq_lane=0;seq_lane<4;seq_lane=seq_lane+1) begin
            lane_bank_q[seq_lane] <= 2'd0;
            lane_byte_q[seq_lane] <= 1'b0;
        end
    end else begin
        if (rd_meta_valid) begin
            rsp_fifo[rsp_wr_ptr] <= assembled_read_data;
            rsp_wr_ptr           <= rsp_wr_ptr + 1'b1;
        end
        if (rsp_pop)
            rsp_rd_ptr <= rsp_rd_ptr + 1'b1;
        case ({rd_meta_valid, rsp_pop})
            2'b10: rsp_count <= rsp_count + 1'b1;
            2'b01: rsp_count <= rsp_count - 1'b1;
            default: rsp_count <= rsp_count;
        endcase

        // SRAM在请求时钟沿更新rdata；下一时钟沿将四个Bank重排为四个byte。
        if (rd_fire) begin
            rd_meta_valid <= 1'b1;
            rd_buffer_select_q <= rd_buffer_select;
            lane_mask_q <= (decode_conflict || decode_oob) ? 4'd0 : rd_mask;
            for (seq_lane=0;seq_lane<4;seq_lane=seq_lane+1) begin
                lane_bank_q[seq_lane] <= lane_bank_d[seq_lane];
                lane_byte_q[seq_lane] <= lane_byte_d[seq_lane];
            end
            if (decode_conflict || decode_oob)
                access_error <= 1'b1;
        end else begin
            rd_meta_valid <= 1'b0;
        end
        if (wr_valid && wr_ready && (!wr_in_range || !wr_aligned))
            access_error <= 1'b1;
    end
end

endmodule
