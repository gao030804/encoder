// ============================================================================
// 中文阅读导引（当前实现）
// 两项256-bit FIFO保存待装载权重。req_pending是待发请求，wait_response是已发未回请求。
// reserved_count同时计算FIFO占用和在途预留，不能只看fifo_count决定是否接受新命令。
// weight_valid/weight_ready表示下游取走FIFO头；wgt_rd_rsp_valid/ready表示SRAM响应进入FIFO。
// ============================================================================
//=============================================================================
// 模块：weight_loader
// 功能：从 Weight SRAM 读取 256-bit 权重块，并用两项 FIFO 实现双缓冲。
//
// FIFO 头部是当前准备装入 Mesh 的权重，另一项可在当前 k_group 计算时
// 预取。SRAM 接口仍只保留一个 outstanding 请求，兼容现有单端口模型。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 把Weight SRAM请求/响应转换为内部256-bit FIFO，隔离SRAM延迟与Mesh装载时机。
// - reserved_count同时统计FIFO数据和已发未回请求，防止流水请求造成FIFO超额预留。
// - weight_valid/ready消费FIFO头，wgt_rd_rsp_valid/ready接收SRAM响应，两组握手含义不同。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：256-bit权重读取缓冲。
// 关键变量/接口：将SRAM请求/响应转换成内部FIFO，reserved_count避免在途响应导致FIFO溢出。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module weight_loader #(
    parameter ADDR_WIDTH = 32
) (
    input                                       clk,
    input                                       rst_n,
    input                                       cmd_valid,
    output                                      cmd_ready,
    input              [ADDR_WIDTH-1:0]         cmd_addr,
    output                                      wgt_rd_req_valid,
    input                                       wgt_rd_req_ready,
    output             [ADDR_WIDTH-1:0]         wgt_rd_addr,
    input                                       wgt_rd_rsp_valid,
    output                                      wgt_rd_rsp_ready,
    input              [255:0]                  wgt_rd_data,
    output                                      weight_valid,
    input                                       weight_ready,
    output             [255:0]                  weight_data
);

reg                  req_pending;
reg                  wait_response;
reg [ADDR_WIDTH-1:0] addr_q;
reg [255:0] weight_fifo [0:1];
reg         fifo_wr_ptr;
reg         fifo_rd_ptr;
reg [1:0]   fifo_count;
wire weight_pop  = weight_valid && weight_ready;
wire weight_push = wgt_rd_rsp_valid && wgt_rd_rsp_ready;

// 已缓存项和在途项都计入保留容量，保证返回时不会溢出。
wire [2:0] reserved_count = {1'b0, fifo_count} +
    (req_pending   ? 3'd1 : 3'd0) +
    (wait_response ? 3'd1 : 3'd0);
// 连续赋值：组合生成cmd_ready及其相邻接口信号，表达握手、选择或地址关系。
assign cmd_ready        = (reserved_count < 3'd2);
assign wgt_rd_req_valid = req_pending;
// 连续赋值：组合生成wgt_rd_addr及其相邻接口信号，表达握手、选择或地址关系。
assign wgt_rd_addr      = addr_q;
assign wgt_rd_rsp_ready = wait_response &&
    ((fifo_count < 2) || weight_pop);
// 连续赋值：组合生成weight_valid及其相邻接口信号，表达握手、选择或地址关系。
assign weight_valid = (fifo_count != 0);
assign weight_data  = weight_fifo[fifo_rd_ptr];

// 时序逻辑：在时钟沿更新req_pending、wait_response、addr_q、fifo_wr_ptr、fifo_rd_ptr、fifo_count；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        req_pending   <= 1'b0;
        wait_response <= 1'b0;
        addr_q        <= {ADDR_WIDTH{1'b0}};
        fifo_wr_ptr   <= 1'b0;
        fifo_rd_ptr   <= 1'b0;
        fifo_count    <= 2'd0;
    end else begin
        if (cmd_valid && cmd_ready) begin
            addr_q      <= cmd_addr;
            req_pending <= 1'b1;
        end
        if (wgt_rd_req_valid && wgt_rd_req_ready) begin
            req_pending   <= 1'b0;
            wait_response <= 1'b1;
        end
        if (weight_push) begin
            weight_fifo[fifo_wr_ptr] <= wgt_rd_data;
            fifo_wr_ptr              <= fifo_wr_ptr + 1'b1;
            wait_response            <= 1'b0;
        end
        if (weight_pop)
            fifo_rd_ptr <= fifo_rd_ptr + 1'b1;
        case ({weight_push, weight_pop})
            2'b10: fifo_count <= fifo_count + 1'b1;
            2'b01: fifo_count <= fifo_count - 1'b1;
            default: fifo_count <= fifo_count;
        endcase
    end
end

endmodule
