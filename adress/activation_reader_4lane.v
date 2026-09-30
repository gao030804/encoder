// ============================================================================
// 中文阅读导引（当前实现）
// 将AGU的4路Byte地址请求送入SRAM，并缓存返回的32-bit激活数据与m标签。
// 请求、返回和Mesh消费彼此解耦；必须为在途请求预留FIFO空间，防止SRAM返回时队列溢出。
// valid且ready在上升沿同时为1才推进相应计数器；Mesh反压时已返回的数据和标签应保持成对。
// ============================================================================
//=============================================================================
// 模块：activation_reader_4lane
// 功能：将 AGU 地址流转换为 Activation SRAM 请求，并缓存返回数据。
//
// 与旧版“一次只能有一个请求”不同，本模块允许多个读请求在途：
//   1. AGU 与 SRAM 请求端直接进行 valid/ready 握手；
//   2. 每个已发请求的 mask 和 m_tag 进入元数据 FIFO；
//   3. SRAM 按请求顺序返回，返回数据与 FIFO 头部元数据配对；
//   4. 结果 FIFO 吸收 Mesh 反压。
//
// 当 SRAM 能够每拍接收/返回一个请求时，本模块稳态也可做到每拍输出一组
// 4xINT8 激活。FIFO_DEPTH 必须为 2 的幂，默认 8 项。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 把AGU产生的4个Byte地址和mask发给Activation SRAM，并把返回数据与m_tag重新对齐。
// - 内部命令/响应缓冲允许多个请求在途；reserved计数包含已发出但尚未返回的请求。
// - 下游停顿时rsp_valid、rsp_data和rsp_m_tag保持稳定，形成完整反压链。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：四路激活SRAM读取流水。
// 关键变量/接口：保存请求tag并跟踪未返回事务；rsp_valid等待rsp_ready时数据必须保持稳定。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module activation_reader_4lane #(
    parameter ADDR_WIDTH  = 32,
    parameter M_TAG_WIDTH = 9,
    parameter FIFO_DEPTH  = 8,
    parameter PTR_WIDTH   = 3
) (
    input                                       clk,
    input                                       rst_n,
    input                                       cmd_valid,
    output                                      cmd_ready,
    input              [ADDR_WIDTH-1:0]         cmd_addr0,
    input              [ADDR_WIDTH-1:0]         cmd_addr1,
    input              [ADDR_WIDTH-1:0]         cmd_addr2,
    input              [ADDR_WIDTH-1:0]         cmd_addr3,
    input              [3:0]                    cmd_mask,
    input              [M_TAG_WIDTH-1:0]        cmd_m_tag,
    output                                      act_rd_req_valid,
    input                                       act_rd_req_ready,
    output             [ADDR_WIDTH-1:0]         act_rd_addr0,
    output             [ADDR_WIDTH-1:0]         act_rd_addr1,
    output             [ADDR_WIDTH-1:0]         act_rd_addr2,
    output             [ADDR_WIDTH-1:0]         act_rd_addr3,
    output             [3:0]                    act_rd_mask,
    input                                       act_rd_rsp_valid,
    output                                      act_rd_rsp_ready,
    input              [63:0]                   act_rd_data,
    output                                      rsp_valid,
    input                                       rsp_ready,
    output             [63:0]                   rsp_data,
    output             [M_TAG_WIDTH-1:0]        rsp_m_tag
);

reg [3:0]             meta_mask [0:FIFO_DEPTH-1];
reg [M_TAG_WIDTH-1:0] meta_tag  [0:FIFO_DEPTH-1];
reg [PTR_WIDTH-1:0]   meta_wr_ptr;
reg [PTR_WIDTH-1:0]   meta_rd_ptr;
reg [PTR_WIDTH:0]     meta_count;
reg [63:0]            out_data [0:FIFO_DEPTH-1];
reg [M_TAG_WIDTH-1:0] out_tag  [0:FIFO_DEPTH-1];
reg [PTR_WIDTH-1:0]   out_wr_ptr;
reg [PTR_WIDTH-1:0]   out_rd_ptr;
reg [PTR_WIDTH:0]     out_count;
wire out_pop  = rsp_valid && rsp_ready;
wire meta_pop = act_rd_rsp_valid && act_rd_rsp_ready;
wire out_has_space  = (out_count < FIFO_DEPTH) || out_pop;
wire meta_has_space = (meta_count < FIFO_DEPTH) || meta_pop;
// 连续赋值：组合生成act_rd_req_valid及其相邻接口信号，表达握手、选择或地址关系。
assign act_rd_req_valid = cmd_valid && meta_has_space;
assign cmd_ready        = act_rd_req_ready && meta_has_space;
// 连续赋值：组合生成act_rd_addr0及其相邻接口信号，表达握手、选择或地址关系。
assign act_rd_addr0     = cmd_addr0;
assign act_rd_addr1     = cmd_addr1;
// 连续赋值：组合生成act_rd_addr2及其相邻接口信号，表达握手、选择或地址关系。
assign act_rd_addr2     = cmd_addr2;
assign act_rd_addr3     = cmd_addr3;
// 连续赋值：组合生成act_rd_mask及其相邻接口信号，表达握手、选择或地址关系。
assign act_rd_mask      = cmd_mask;
wire req_fire = act_rd_req_valid && act_rd_req_ready;

// SRAM 必须按请求顺序返回；没有对应元数据时不接受孤立响应。
// 连续赋值：组合生成act_rd_rsp_ready及其相邻接口信号，表达握手、选择或地址关系。
assign act_rd_rsp_ready = (meta_count != 0) && out_has_space;
assign rsp_valid = (out_count != 0);
// 连续赋值：组合生成rsp_data及其相邻接口信号，表达握手、选择或地址关系。
assign rsp_data  = out_data[out_rd_ptr];
assign rsp_m_tag = out_tag[out_rd_ptr];
wire [63:0] masked_rsp_data = {
meta_mask[meta_rd_ptr][3] ? act_rd_data[63:48] : 16'd0,
meta_mask[meta_rd_ptr][2] ? act_rd_data[47:32] : 16'd0,
meta_mask[meta_rd_ptr][1] ? act_rd_data[31:16] : 16'd0,
meta_mask[meta_rd_ptr][0] ? act_rd_data[15:0]  : 16'd0
};

// 时序逻辑：在时钟沿更新meta_wr_ptr、meta_rd_ptr、meta_count、out_wr_ptr、out_rd_ptr、out_count；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        meta_wr_ptr <= {PTR_WIDTH{1'b0}};
        meta_rd_ptr <= {PTR_WIDTH{1'b0}};
        meta_count  <= {(PTR_WIDTH+1){1'b0}};
        out_wr_ptr  <= {PTR_WIDTH{1'b0}};
        out_rd_ptr  <= {PTR_WIDTH{1'b0}};
        out_count   <= {(PTR_WIDTH+1){1'b0}};
    end else begin
        if (req_fire) begin
            meta_mask[meta_wr_ptr] <= cmd_mask;
            meta_tag[meta_wr_ptr]  <= cmd_m_tag;
            meta_wr_ptr            <= meta_wr_ptr + 1'b1;
        end
        if (meta_pop) begin
            out_data[out_wr_ptr] <= masked_rsp_data;
            out_tag[out_wr_ptr]  <= meta_tag[meta_rd_ptr];
            out_wr_ptr           <= out_wr_ptr + 1'b1;
            meta_rd_ptr          <= meta_rd_ptr + 1'b1;
        end
        case ({req_fire, meta_pop})
            2'b10: meta_count <= meta_count + 1'b1;
            2'b01: meta_count <= meta_count - 1'b1;
            default: meta_count <= meta_count;
        endcase
        if (out_pop)
            out_rd_ptr <= out_rd_ptr + 1'b1;
        case ({meta_pop, out_pop})
            2'b10: out_count <= out_count + 1'b1;
            2'b01: out_count <= out_count - 1'b1;
            default: out_count <= out_count;
        endcase
    end
end

endmodule
