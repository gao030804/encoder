`timescale 1ns/1ns

//=============================================================================
// 方案A权重层次测试：Flash一次Boot -> Global SRAM -> Local A/B -> NPU读取。
// 为缩短行为级Flash仿真，DUT容量保持方案A结构，但只装入12个测试Block。
//=============================================================================
module tb_encoder_weight_hierarchy;

reg clk;
reg rst_n;
reg req_valid;
wire req_ready;
reg [31:0] req_addr;
wire rsp_valid;
reg rsp_ready;
wire [255:0] rsp_data;

reg prefetch_valid;
wire prefetch_ready;
reg [23:0] prefetch_global_base;
reg [15:0] prefetch_block_count;
reg prefetch_target_bank;
wire prefetch_done;

reg activate_valid;
wire activate_ready;
reg activate_first_layer;
reg activate_bank;
wire active_first_layer;
wire active_bank;
wire active_bank_valid;
wire first_layer_ready;
wire hierarchy_busy;
wire hierarchy_error;
wire global_sram_active;
wire local_sram_a_active;
wire local_sram_b_active;

wire flash_csn;
wire flash_sclk;
tri flash_mosi;
tri flash_miso;
tri flash_wpn;
tri flash_holdn;

integer error_count;
integer timeout_count;
integer byte_index;
integer cs_fall_count;
integer global_write_count;
integer local_a_write_count;
integer local_b_write_count;
reg [255:0] held_data;

encoder_weight_hierarchy #(
    .GLOBAL_DEPTH_BLOCKS      (7168),
    .GLOBAL_INDEX_WIDTH       (13),
    .GLOBAL_MODEL_BLOCKS      (12),
    // 缩小Local深度以便在短仿真中覆盖“大层分Tile/Demand refill”逻辑。
    .LOCAL_DEPTH_BLOCKS       (4),
    .LOCAL_INDEX_WIDTH        (2),
    .FIRST_LAYER_FLASH_BASE   (24'h000000),
    .FIRST_LAYER_BLOCKS       (4),
    .QSPI_HALF_DIV            (16'd1),
    .FLASH_POWER_UP_CYCLES    (32'd20),
    .QSPI_DUMMY_CYCLES        (8)
) dut (
    .clk(clk), .rst_n(rst_n),
    .req_valid(req_valid), .req_ready(req_ready),
    .req_addr(req_addr), .rsp_valid(rsp_valid),
    .rsp_ready(rsp_ready), .rsp_data(rsp_data),
    .prefetch_valid(prefetch_valid), .prefetch_ready(prefetch_ready),
    .prefetch_global_base(prefetch_global_base),
    .prefetch_block_count(prefetch_block_count),
    .prefetch_target_bank(prefetch_target_bank),
    .prefetch_done(prefetch_done),
    .activate_valid(activate_valid), .activate_ready(activate_ready),
    .activate_first_layer(activate_first_layer),
    .activate_bank(activate_bank),
    .active_first_layer(active_first_layer),
    .active_bank(active_bank), .active_bank_valid(active_bank_valid),
    .first_layer_ready(first_layer_ready),
    .hierarchy_busy(hierarchy_busy), .hierarchy_error(hierarchy_error),
    .global_sram_active(global_sram_active),
    .local_sram_a_active(local_sram_a_active),
    .local_sram_b_active(local_sram_b_active),
    .flash_csn(flash_csn), .flash_sclk(flash_sclk),
    .flash_mosi(flash_mosi), .flash_miso(flash_miso),
    .flash_wp(flash_wpn), .flash_hold(flash_holdn)
);

P25Q32H #(
    .Init_File("tb/data/flash/hierarchy_12blocks.hex")
) u_flash (
    .SCLK(flash_sclk), .CSb(flash_csn),
    .SI(flash_mosi), .SO(flash_miso),
    .WPb(flash_wpn), .SIO3(flash_holdn)
);

initial clk = 1'b0;
always #5 clk = ~clk;

always @(negedge flash_csn)
    cs_fall_count = cs_fall_count + 1;

always @(posedge clk) begin
    if (dut.global_dma_write)
        global_write_count = global_write_count + 1;
    if (dut.bank_a_sram_write)
        local_a_write_count = local_a_write_count + 1;
    if (dut.bank_b_sram_write)
        local_b_write_count = local_b_write_count + 1;

    if (prefetch_valid && prefetch_ready)
        $display("[TRACE] G2L accept time=%0t target=%s base=%06h blocks=%0d",
                 $time, prefetch_target_bank ? "B" : "A",
                 prefetch_global_base, prefetch_block_count);
    if (prefetch_done)
        $display("[TRACE] G2L done   time=%0t writes(A/B)=%0d/%0d",
                 $time, local_a_write_count, local_b_write_count);
end

task issue_prefetch;
    input [23:0] base;
    input [15:0] count;
    input bank;
    begin : prefetch_body
        @(negedge clk);
        prefetch_global_base = base;
        prefetch_block_count = count;
        prefetch_target_bank = bank;
        prefetch_valid       = 1'b1;
        timeout_count = 0;
        @(posedge clk);
        while (!prefetch_ready && timeout_count < 1000) begin
            @(posedge clk);
            timeout_count = timeout_count + 1;
        end
        if (!prefetch_ready) begin
            $display("[ERROR] G2L command timeout base=%06h", base);
            error_count = error_count + 1;
            disable prefetch_body;
        end
        @(negedge clk);
        prefetch_valid = 1'b0;

        timeout_count = 0;
        while (!prefetch_done && timeout_count < 2000) begin
            @(posedge clk);
            timeout_count = timeout_count + 1;
        end
        if (!prefetch_done) begin
            $display("[ERROR] G2L completion timeout base=%06h", base);
            error_count = error_count + 1;
        end
    end
endtask

task activate_source;
    input first;
    input bank;
    begin : activate_body
        @(negedge clk);
        activate_first_layer = first;
        activate_bank        = bank;
        activate_valid       = 1'b1;
        timeout_count = 0;
        @(posedge clk);
        while (!activate_ready && timeout_count < 1000) begin
            @(posedge clk);
            timeout_count = timeout_count + 1;
        end
        if (!activate_ready) begin
            $display("[ERROR] activate timeout first=%0b bank=%0b", first, bank);
            error_count = error_count + 1;
            disable activate_body;
        end
        @(negedge clk);
        activate_valid = 1'b0;
    end
endtask

task verify_block;
    input [31:0] address;
    input integer first_byte;
    input integer stall_cycles;
    begin : verify_body
        @(negedge clk);
        req_addr  = address;
        req_valid = 1'b1;
        timeout_count = 0;
        @(posedge clk);
        while (!req_ready && timeout_count < 1000) begin
            @(posedge clk);
            timeout_count = timeout_count + 1;
        end
        if (!req_ready) begin
            $display("[ERROR] Local read request timeout addr=%08h", address);
            error_count = error_count + 1;
            disable verify_body;
        end
        @(negedge clk);
        req_valid = 1'b0;

        timeout_count = 0;
        while (!rsp_valid && timeout_count < 1000) begin
            @(posedge clk);
            timeout_count = timeout_count + 1;
        end
        if (!rsp_valid) begin
            $display("[ERROR] Local read response timeout addr=%08h", address);
            error_count = error_count + 1;
        end else begin
            held_data = rsp_data;
            repeat (stall_cycles) begin
                @(posedge clk);
                if (!rsp_valid || (rsp_data !== held_data)) begin
                    $display("[ERROR] response changed under backpressure");
                    error_count = error_count + 1;
                end
            end
            for (byte_index = 0; byte_index < 32; byte_index = byte_index + 1) begin
                if (rsp_data[8*byte_index +: 8] !==
                    ((first_byte + byte_index) & 8'hff)) begin
                    $display("[ERROR] addr=%08h byte=%0d exp=%02h got=%02h",
                             address, byte_index,
                             ((first_byte + byte_index) & 8'hff),
                             rsp_data[8*byte_index +: 8]);
                    error_count = error_count + 1;
                end
            end
            @(negedge clk);
            rsp_ready = 1'b1;
            @(posedge clk);
            @(negedge clk);
            rsp_ready = 1'b0;
            $display("[CHECK] Local data matches Global/Flash addr=%08h", address);
        end
    end
endtask

initial begin
    rst_n = 1'b0;
    req_valid = 1'b0;
    req_addr = 32'd0;
    rsp_ready = 1'b0;
    prefetch_valid = 1'b0;
    prefetch_global_base = 24'd0;
    prefetch_block_count = 16'd0;
    prefetch_target_bank = 1'b0;
    activate_valid = 1'b0;
    activate_first_layer = 1'b0;
    activate_bank = 1'b0;
    error_count = 0;
    timeout_count = 0;
    cs_fall_count = 0;
    global_write_count = 0;
    local_a_write_count = 0;
    local_b_write_count = 0;

    repeat (5) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;

    // Boot只允许一次Flash Burst；完成后第一层4个Block已复制到常驻Local。
    while (!first_layer_ready && timeout_count < 500000) begin
        @(posedge clk);
        timeout_count = timeout_count + 1;
    end
    if (!first_layer_ready) begin
        $display("[ERROR] Flash-to-Global boot timeout");
        error_count = error_count + 1;
    end
    $display("[TRACE] Boot done time=%0t Global writes=%0d Flash CS=%0d",
             $time, global_write_count, cs_fall_count);

    verify_block(32'h0000_0000, 8'h00, 2);
    verify_block(32'h0000_0060, 8'h60, 0);

    // Flash已经休眠/空闲。以下数据全部来自Global SRAM，不得再次拉低Flash CS。
    // 8个Block大于测试Local Bank的4个Block：首Tile进入B，后续读触发自动换Tile。
    issue_prefetch(24'h000080, 16'd8, 1'b1);
    activate_source(1'b0, 1'b1);
    verify_block(32'h0000_0080, 8'h80, 3);
    verify_block(32'h0000_0100, 8'h00, 0);
    verify_block(32'h0000_0120, 8'h20, 0);

    issue_prefetch(24'h000040, 16'd2, 1'b1);
    activate_source(1'b0, 1'b1);
    verify_block(32'h0000_0040, 8'h40, 1);

    // 帧结束切回第一层，仍不访问Flash。
    activate_source(1'b1, 1'b0);
    verify_block(32'h0000_0020, 8'h20, 0);

    if (global_write_count != 12) begin
        $display("[ERROR] Global write count expected=12 actual=%0d",
                 global_write_count);
        error_count = error_count + 1;
    end
    if ((local_b_write_count != 6) || (local_a_write_count != 4)) begin
        $display("[ERROR] Local write count expected A/B=4/6 actual=%0d/%0d",
                 local_a_write_count, local_b_write_count);
        error_count = error_count + 1;
    end
    // QE配置使用两次CS事务，数据仅一次连续Fast Read；总CS=3，不是3次读数据。
    if (cs_fall_count != 3) begin
        $display("[ERROR] Flash boot expected 2 setup + 1 burst transactions; CS count=%0d",
                 cs_fall_count);
        error_count = error_count + 1;
    end
    if (hierarchy_error) begin
        $display("[ERROR] hierarchy_error asserted");
        error_count = error_count + 1;
    end

    // 高位地址曾可能截断成13-bit块号而别名到低地址；必须拒绝且不发起SRAM读。
    @(negedge clk);req_valid=1'b1;req_addr=32'h0004_0020;
    repeat(4) begin
        @(posedge clk);#1;
        if(req_ready || rsp_valid || dut.global_copy_read) begin
            $display("[ERROR] out-of-range Global address was accepted");
            error_count=error_count+1;
        end
    end
    @(negedge clk);req_valid=1'b0;
    if(!hierarchy_error) begin
        $display("[ERROR] out-of-range Global address did not raise error");
        error_count=error_count+1;
    end else $display("[GLOBAL_OOB_PASS] full byte address checked without truncation alias");

    if (error_count == 0)
        $display("[TB_PASS] Flash-once + Global SRAM + Local A/B hierarchy correct");
    else
        $display("[TB_FAIL] errors=%0d", error_count);

    $finish;
end

endmodule
