//=============================================================================
// 模块：qspi_weight_boot_loader
// 功能：P25Q32H 上电初始化与 QSPI 连续突发读取。
//
// 时钟关系（本编码器主时钟50 MHz）：
//   QSPI_HALF_DIV=1，flash_sclk 每个主时钟翻转一次，得到25 MHz SCLK。
//
// 启动流程：
//   1. 等待 P25Q32H 上电稳定；
//   2. 发送 50h（Volatile SR Write Enable）；
//   3. 发送 31h + 02h，置位状态寄存器2中的 QE；
//   4. CS保持为低，只发送一次 6Bh + 24-bit地址，再产生dummy时钟；
//   5. IO[3:0] 每个SCLK传回4 bit，连续拼成256-bit权重块。
//
// 数据排布：Flash低地址字节放入 block_data[7:0]，与Global SRAM字节地址一致。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 从P25Q32H执行上电初始化、设置QE并发送6Bh Quad Output Fast Read命令。
// - CS在整个权重镜像Burst期间保持有效，返回的4-bit nibble连续拼装成Byte，再拼成256-bit块。
// - block_valid/ready支持Global SRAM反压；block_index只在完整32-Byte块被接受时递增。
// - QSPI_HALF_DIV=1且系统时钟50 MHz时，输出SCLK为25 MHz。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：QSPI上电权重加载器。
// 关键变量/接口：以25MHz Quad Fast Read连续读取Flash，按byte再按256-bit block组装后写Global SRAM。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module qspi_weight_boot_loader #(
    parameter QSPI_HALF_DIV       = 16'd1,
    parameter POWER_UP_CYCLES     = 32'd7000,
    parameter QSPI_DUMMY_CYCLES   = 8,

    // P25Q32H要求相邻命令之间CS#高电平至少保持30 ns。
    parameter CS_HIGH_CYCLES      = 3,
    parameter TIMEOUT_CYCLES      = 32'd20000000
) (
    input                                       clk,
    input                                       rst_n,
    input                                       cmd_valid,
    output                                      cmd_ready,
    input              [23:0]                   cmd_flash_base,
    input              [15:0]                   cmd_block_count,
    output reg                                  block_valid,
    input                                       block_ready,
    output reg         [15:0]                   block_index,
    output reg         [255:0]                  block_data,
    output reg                                  done,
    output reg                                  busy,
    output reg                                  error,
    output reg                                  flash_csn,
    output reg                                  flash_sclk,
    inout              [3:0]                    flash_dq
);

localparam [3:0]
S_POWER_WAIT = 4'd0,
S_VWREN      = 4'd1,
S_VWREN_END  = 4'd2,
S_WRSR2      = 4'd3,
S_WRSR2_END  = 4'd4,
S_IDLE       = 4'd5,
S_PREFIX     = 4'd6,
S_DUMMY      = 4'd7,
S_READ       = 4'd8,
S_BLOCK      = 4'd9,
S_FINISH     = 4'd10,
S_ERROR      = 4'd11;
reg [3:0]  state;
reg [31:0] wait_count;
reg [7:0]  cs_high_count;
reg [31:0] timeout_count;
reg [15:0] half_div_count;
reg [5:0]  bit_count;
reg [5:0]  nibble_count;
reg [39:0] prefix_shift;
reg [15:0] block_count_q;
reg [255:0] data_shift;
reg [3:0]  dq_out;
reg [3:0]  dq_oe;
wire [3:0] dq_in = flash_dq;
// 连续赋值：组合生成flash_dq及其相邻接口信号，表达握手、选择或地址关系。
assign flash_dq[0] = dq_oe[0] ? dq_out[0] : 1'bz;
assign flash_dq[1] = dq_oe[1] ? dq_out[1] : 1'bz;
// 连续赋值：组合生成flash_dq及其相邻接口信号，表达握手、选择或地址关系。
assign flash_dq[2] = dq_oe[2] ? dq_out[2] : 1'bz;
assign flash_dq[3] = dq_oe[3] ? dq_out[3] : 1'bz;
// 连续赋值：组合生成cmd_ready及其相邻接口信号，表达握手、选择或地址关系。
assign cmd_ready = (state == S_IDLE);

// SCLK只在串行事务状态翻转；其余时间固定为0。
wire serial_active = (state == S_VWREN) || (state == S_WRSR2) ||
    (state == S_PREFIX) || (state == S_DUMMY) ||
    (state == S_READ);
wire half_tick = serial_active && (half_div_count == QSPI_HALF_DIV - 1'b1);
wire rising_tick  = half_tick && !flash_sclk;
wire falling_tick = half_tick &&  flash_sclk;

// 时序逻辑：在时钟沿更新state、wait_count、cs_high_count、timeout_count、half_div_count、bit_count；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state          <= S_POWER_WAIT;
        wait_count     <= 32'd0;
        cs_high_count  <= 8'd0;
        timeout_count  <= 32'd0;
        half_div_count <= 16'd0;
        bit_count      <= 6'd0;
        nibble_count   <= 6'd0;
        prefix_shift   <= 40'd0;
        block_count_q  <= 16'd0;
        data_shift     <= 256'd0;
        dq_out         <= 4'hf;
        dq_oe          <= 4'b0000;
        block_valid    <= 1'b0;
        block_index    <= 16'd0;
        block_data     <= 256'd0;
        done           <= 1'b0;
        busy           <= 1'b1;
        error          <= 1'b0;
        flash_csn      <= 1'b1;
        flash_sclk     <= 1'b0;
    end else begin
        done <= 1'b0;
        if (serial_active) begin
            if (half_tick) begin
                half_div_count <= 16'd0;
                flash_sclk     <= ~flash_sclk;
            end else begin
                half_div_count <= half_div_count + 1'b1;
            end
        end else begin
            half_div_count <= 16'd0;
            flash_sclk     <= 1'b0;
        end
        if (busy && (state != S_POWER_WAIT) && (state != S_BLOCK)) begin
            if (timeout_count >= TIMEOUT_CYCLES - 1'b1) begin
                error     <= 1'b1;
                flash_csn <= 1'b1;
                dq_oe     <= 4'b0000;
                state     <= S_ERROR;
            end else begin
                timeout_count <= timeout_count + 1'b1;
            end
        end else begin
            timeout_count <= 32'd0;
        end
        case (state)
            S_POWER_WAIT: begin
                busy       <= 1'b1;
                flash_csn  <= 1'b1;
                flash_sclk <= 1'b0;
                dq_oe      <= 4'b0000;
                if (wait_count >= POWER_UP_CYCLES - 1'b1) begin
                    wait_count <= 32'd0;
                    flash_csn  <= 1'b0;
                    dq_oe      <= 4'b1101; // IO0发命令；IO2/3保持高，IO1释放。
                    dq_out     <= 4'b1100;
                    prefix_shift <= {8'h50, 32'd0};
                    bit_count   <= 6'd0;
                    state       <= S_VWREN;
                end else begin
                    wait_count <= wait_count + 1'b1;
                end
            end

            // 50h与31h/02h均走单线命令阶段，Flash在SCLK上升沿采样IO0。
            S_VWREN: begin
                if (falling_tick) begin
                    if (bit_count == 6'd7) begin
                        flash_csn <= 1'b1;
                        cs_high_count <= 8'd0;
                        state     <= S_VWREN_END;
                    end else begin
                        bit_count <= bit_count + 1'b1;
                        dq_out[0] <= prefix_shift[38-bit_count];
                    end
                end
            end
            S_VWREN_END: begin
                flash_csn <= 1'b1;
                dq_oe     <= 4'b0000;
                if (cs_high_count >= CS_HIGH_CYCLES - 1) begin
                    flash_csn     <= 1'b0;
                    dq_oe         <= 4'b1101;
                    prefix_shift  <= {8'h31, 8'h02, 24'd0};
                    dq_out[0]     <= 1'b0;
                    bit_count     <= 6'd0;
                    cs_high_count <= 8'd0;
                    state         <= S_WRSR2;
                end else begin
                    cs_high_count <= cs_high_count + 1'b1;
                end
            end
            S_WRSR2: begin
                if (falling_tick) begin
                    if (bit_count == 6'd15) begin
                        flash_csn <= 1'b1;
                        cs_high_count <= 8'd0;
                        state     <= S_WRSR2_END;
                    end else begin
                        bit_count <= bit_count + 1'b1;
                        dq_out[0] <= prefix_shift[38-bit_count];
                    end
                end
            end
            S_WRSR2_END: begin
                flash_csn <= 1'b1;
                dq_oe     <= 4'b0000;
                if (cs_high_count >= CS_HIGH_CYCLES - 1) begin
                    busy          <= 1'b0;
                    cs_high_count <= 8'd0;
                    state         <= S_IDLE;
                end else begin
                    cs_high_count <= cs_high_count + 1'b1;
                end
            end
            S_IDLE: begin
                busy <= 1'b0;
                if (cmd_valid && cmd_ready) begin
                    if ((cmd_block_count == 0) ||
                        ({17'd0, cmd_flash_base} + ({25'd0, cmd_block_count} << 5) >
                        41'h0_0040_0000)) begin
                        error <= 1'b1;
                        state <= S_ERROR;
                    end else begin
                        busy          <= 1'b1;
                        flash_csn    <= 1'b0;
                        dq_oe        <= 4'b1101;
                        prefix_shift <= {8'h6b, cmd_flash_base, 8'h00};
                        dq_out[0]    <= 1'b0;
                        bit_count     <= 6'd0;
                        block_count_q <= cmd_block_count;
                        block_index   <= 16'd0;
                        state         <= S_PREFIX;
                    end
                end
            end
            S_PREFIX: begin
                if (falling_tick) begin

                    // 6Bh命令和24-bit地址共32 bit；dummy阶段释放全部IO。
                    if (bit_count == 6'd31) begin
                        bit_count <= 6'd0;
                        dq_oe     <= 4'b0000;
                        state     <= S_DUMMY;
                    end else begin
                        bit_count <= bit_count + 1'b1;
                        dq_out[0] <= prefix_shift[38-bit_count];
                    end
                end
            end
            S_DUMMY: begin
                if (falling_tick) begin
                    if (bit_count == QSPI_DUMMY_CYCLES - 1) begin
                        bit_count    <= 6'd0;
                        nibble_count <= 6'd0;
                        data_shift   <= 256'd0;
                        state        <= S_READ;
                    end else begin
                        bit_count <= bit_count + 1'b1;
                    end
                end
            end
            S_READ: begin

                // Flash在下降沿更新DQ，控制器在下一上升沿采样，满足建立时间。
                if (rising_tick) begin
                    if (nibble_count[0])
                        data_shift[8*(nibble_count >> 1) +: 4] <= dq_in;
                    else
                    data_shift[8*(nibble_count >> 1) + 4 +: 4] <= dq_in;
                    if (nibble_count == 6'd63) begin
                        block_data  <= data_shift |
                            ({{252{1'b0}}, dq_in} << 248);
                        block_valid <= 1'b1;
                        state       <= S_BLOCK;
                    end else begin
                        nibble_count <= nibble_count + 1'b1;
                    end
                end
            end
            S_BLOCK: begin
                if (block_valid && block_ready) begin
                    block_valid <= 1'b0;
                    if (block_index + 1'b1 >= block_count_q) begin
                        flash_csn <= 1'b1;
                        state     <= S_FINISH;
                    end else begin
                        block_index  <= block_index + 1'b1;
                        nibble_count <= 6'd0;
                        data_shift   <= 256'd0;
                        state        <= S_READ;
                    end
                end
            end
            S_FINISH: begin
                busy <= 1'b0;
                done <= 1'b1;
                state <= S_IDLE;
            end
            S_ERROR: begin
                busy       <= 1'b0;
                flash_csn  <= 1'b1;
                flash_sclk <= 1'b0;
                dq_oe      <= 4'b0000;
            end
            default: state <= S_ERROR;
        endcase
    end
end

endmodule
