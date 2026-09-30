`timescale 1ns/1ps
// 256条x128bit、单时钟1写1读输出队列。容量4KiB，50frame/s时保存5.12秒。
// 一条记录：{timestamp_us[31:0], frame_id[31:0], indices[63:0]}。
// 同步SRAM读回后进入保持寄存器；DMA未接收前，不释放SRAM对应槽位。
// 因此level包含已预读的队首记录，保证总容量严格为256条。
module encoded_output_ring (
    input clk,
    input rst_n,
    input in_valid,
    output in_ready,
    input [63:0] in_indices,
    input [31:0] in_frame_id,
    input [31:0] in_timestamp_us,
    output reg dma_valid,
    input dma_ready,
    output reg [127:0] dma_data,
    output [15:0] dma_keep,
    output dma_last,
    output reg [8:0] level,
    output full,
    output almost_full,
    output reg stall_seen
);
// 指针单位为一条128bit记录，不是字节地址；8bit加法自动模256。
reg [7:0] write_pointer, read_pointer;
reg read_pending; // 已发出同步SRAM读请求，但尚未锁存响应
wire push = in_valid && in_ready;     // 真正写入SRAM的握手
wire pop  = dma_valid && dma_ready;   // DMA真正接收并释放队首槽位
wire read_request = (level != 0) && !dma_valid && !read_pending;
wire [127:0] read_data;
wire read_valid;

// 满时保留一拍保守反压，DMA弹出后再接收新记录，避免依赖同地址读写语义。
assign full = (level == 9'd256);
assign almost_full = (level >= 9'd224);
assign in_ready = rst_n && !full;
assign dma_keep = 16'hffff;
assign dma_last = 1'b1; // 每条16Byte记录作为一个DMA stream packet

opentitan_sram_1r1w_adapter #(
    .WIDTH(128), .DEPTH(256), .DATA_BITS_PER_MASK(8)
) u_record_sram (
    .clk_i(clk), .rst_ni(rst_n),
    .wr_req_i(push), .wr_addr_i(write_pointer),
    .wr_data_i({in_timestamp_us,in_frame_id,in_indices}), .wr_mask_i(16'hffff),
    .rd_req_i(read_request), .rd_addr_i(read_pointer),
    .rd_data_o(read_data), .rd_valid_o(read_valid)
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        write_pointer <= 8'd0;
        read_pointer  <= 8'd0;
        level         <= 9'd0;
        read_pending  <= 1'b0;
        dma_valid     <= 1'b0;
        dma_data      <= 128'd0;
        stall_seen    <= 1'b0;
    end else begin
        if (push) write_pointer <= write_pointer + 1'b1;
        if (pop)  read_pointer  <= read_pointer + 1'b1;
        // 同拍一写一读时占用数不变；不能在预读时提前减level。
        case ({push, pop})
            2'b10:   level <= level + 1'b1;
            2'b01:   level <= level - 1'b1;
            default: level <= level;
        endcase
        // 8bit指针自然从255回绕到0。复位仅丢弃队列，不逐字清SRAM。
        if (read_request) read_pending <= 1'b1;
        if (read_valid) begin
            read_pending <= 1'b0;
            dma_data     <= read_data;
            dma_valid    <= 1'b1;
        end
        // 无新响应且DMA不接收时不赋值，保持valid/data直至握手。
        if (pop) dma_valid <= 1'b0;
        // 这是正常反压的粘滞观测位，不表示数据已丢失。
        if (in_valid && !in_ready) stall_seen <= 1'b1;
    end
end
endmodule
