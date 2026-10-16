`timescale 1ns/1ps

//=============================================================================
// 上电Flash -> SRAM -> 一帧PCM16 -> 63个物理卷积 -> 1x64 INT16 latent。
//
// 数值检查：每次Activation写回均与Python整数golden逐byte比较。
// 性能检查：记录每层卷积周期、下一层权重预取周期以及预取等待周期。
//=============================================================================
module tb_flash_encoder_rvq_full_frame #(
    parameter integer FRAME_COUNT = 1 // 1保留原回归，10执行独立帧无复位回归
);
`define HIER u_dut.u_encoder.u_engine_flash.g_encoder_weight_hierarchy.u_weight_hierarchy

localparam [31:0] BUFFER_A_BASE = 32'h0000_1000;
localparam [31:0] BUFFER_B_BASE = 32'h0000_5000;
// 50 MHz主时钟；QSPI半周期分频值1，因此实际Flash SCLK为25 MHz。
localparam integer CLOCK_PERIOD_NS = 20;
localparam integer CLOCK_HZ = 50000000;
localparam integer CYCLES_PER_US = 1000 / CLOCK_PERIOD_NS;
localparam integer MAX_TEST_CYCLES = 50000000;
localparam integer NUM_LAYERS = 63;
localparam integer GOLDEN_BYTES = NUM_LAYERS * 8192;
localparam integer EXPECTED_COMPARISONS = 139136;
`ifdef FULL_ENCODER_REAL_SPI
localparam integer USE_REAL_SPI = 1;
`else
localparam integer USE_REAL_SPI = 0;
`endif
`ifdef FULL_ENCODER_ONCHIP_ONLY
localparam integer ONCHIP_ONLY = 1;
`else
localparam integer ONCHIP_ONLY = 0;
`endif

reg clk, rst_n;
reg encoder_start;
wire encoder_start_ready, encoder_busy, encoder_done, encoder_error;

wire input_wr_valid, input_wr_ready;
wire [31:0] input_wr_addr;
wire [63:0] input_wr_data;
wire [7:0] input_wr_strb;
wire input_buffer_select;
wire [31:0] input_buffer_base;

wire layer_prepare_valid;
reg layer_prepare_ready;
reg [119:0] residual_params_mem [0:63];
wire [5:0] physical_layer, logical_layer;
reg elu_lut_cfg_valid;
wire elu_lut_cfg_ready;
reg [6:0] elu_lut_cfg_addr;
reg [7:0] elu_lut_cfg_wdata;

wire param_rd_req_valid;
wire param_rd_req_ready;
wire [5:0] param_rd_layer;
wire [5:0] param_rd_output_group;
wire param_rd_rsp_valid;
wire param_rd_rsp_ready;
wire [511:0] param_bias_data;
wire [255:0] param_mult_data;
wire [63:0] param_shift_data;
wire [63:0] param_zero_point_data;

wire layer_write_fire;
wire [31:0] layer_write_addr;
wire [63:0] layer_write_data;
wire [7:0] layer_write_strb;
wire perf_engine_start, perf_engine_done;
wire perf_weight_sram_read_fire;
wire perf_weight_prefetch_fire, perf_weight_prefetch_done;
wire [23:0] perf_weight_prefetch_base;
wire [15:0] perf_weight_prefetch_blocks;

wire flash_csn, flash_sclk;
tri flash_mosi, flash_miso;
tri1 flash_wpn, flash_holdn;
wire octal_cmd_valid_unused;
wire [23:0] octal_cmd_flash_base_unused;
wire [23:0] octal_cmd_byte_count_unused;
wire octal_rx_ready_unused;

// PCM frontend
reg pcm_frontend_start;
wire pcm_frontend_busy, pcm_frontend_done;
reg pcm_valid;
wire pcm_ready;
reg signed [15:0] pcm_data;
reg [15:0] pcm_memory [0:319];
integer pcm_index;

// Golden memories
reg [7:0] elu_lut [0:127];
reg [15:0] golden_layers [0:GOLDEN_BYTES-1];
reg [15:0] expected_pcm_activation [0:319];
reg [511:0] parameter_bias [0:2047];
reg [255:0] parameter_mult [0:2047];
reg [63:0] parameter_shift [0:2047];
reg [63:0] parameter_zero_point [0:2047];
reg [31:0] expected_layer_elements [0:62];
reg [31:0] pcm_multiplier_mem [0:0];
reg [7:0] pcm_shift_mem [0:0];

integer cycle_count;
integer frame_start_cycle;
integer encoder_start_cycle;
integer encoder_done_cycle;
integer boot_weight_done_cycle;
integer errors;
integer compare_count;
integer pcm_compare_count;
integer timeout_count;
integer lane;
integer layer;
integer offset;
integer golden_index;
integer timing_file;
integer expected_output_bytes [0:NUM_LAYERS-1];
integer actual_output_bytes [0:NUM_LAYERS-1];
integer compute_start_cycle [0:NUM_LAYERS-1];
integer compute_done_cycle [0:NUM_LAYERS-1];
integer first_weight_read_cycle [0:NUM_LAYERS-1];
integer weight_sram_read_count [0:NUM_LAYERS-1];
integer prefetch_start_cycle [0:NUM_LAYERS-1];
integer prefetch_done_cycle [0:NUM_LAYERS-1];
integer prefetch_blocks [0:NUM_LAYERS-1];
integer prefetch_pending_layer;
integer expected_base;
integer transfer_cycles;
integer compute_cycles;
integer slack_cycles;
integer wait_cycles;
integer layer_start_errors [0:NUM_LAYERS-1];
integer sum_compute_cycles;
integer control_overhead_cycles;
integer expected_weight_reads;
integer frame_number=0, frame_errors;
reg [31:0] frame_input_base;
`include "tb/flash_encoder_rvq_support.vh"
`include "tb/flash_encoder_rvq_checks.vh"
`include "tb/flash_encoder_rvq_multiframe.vh"

soundstream_encoder_rvq_w8a16_top #(
    .BUFFER_A_BASE(BUFFER_A_BASE),
    .BUFFER_B_BASE(BUFFER_B_BASE),
    .FLASH_SPI_DIVIDER(16'd1),
    .CLOCK_HZ(CLOCK_HZ),
    .FAST_SIM_WEIGHT_CACHE(0),
    .USE_GLOBAL_WEIGHT_HIERARCHY(1),
    .GLOBAL_MODEL_BLOCKS(6912),
    .BOOT_FLASH_BASE(24'h010000),
    .RESIDUAL_SCALE_ALIGN_ENABLE(1)
) u_dut (
    // 功能模式显式关闭扫描旁路，普通wire输入不能悬空。
    .clk(clk),.rst_n(rst_n),.mesh_scan_test_en(1'b0),.start(encoder_start),
    .start_ready(encoder_start_ready),.busy(encoder_busy),
    .encoder_done(encoder_done),.done(codec_enqueued),
    .encoded_valid(codec_valid),.dma_ready(codec_ready),
    .dma_data(codec_record),.dma_keep(codec_keep),.dma_last(codec_last),
    .output_level(codec_level),
    .encoded_indices(codec_indices),.quantized_vector(codec_vector),.error(encoder_error),
    .input_wr_valid(input_wr_valid),.input_wr_ready(input_wr_ready),
    .input_wr_addr(input_wr_addr),.input_wr_data(input_wr_data),
    .input_wr_strb(input_wr_strb),.input_buffer_select(input_buffer_select),
    .input_buffer_base(input_buffer_base),
    .layer_prepare_valid(layer_prepare_valid),.layer_prepare_ready(layer_prepare_ready),
    .residual_scale_params(residual_params_mem[physical_layer]),
    .physical_layer(physical_layer),.logical_layer(logical_layer),
    .elu_lut_cfg_valid(elu_lut_cfg_valid),.elu_lut_cfg_ready(elu_lut_cfg_ready),
    .elu_lut_cfg_addr(elu_lut_cfg_addr),.elu_lut_cfg_wdata(elu_lut_cfg_wdata),
    .param_rd_req_valid(param_rd_req_valid),.param_rd_req_ready(param_rd_req_ready),
    .param_rd_layer(param_rd_layer),.param_rd_output_group(param_rd_output_group),
    .param_rd_rsp_valid(param_rd_rsp_valid),.param_rd_rsp_ready(param_rd_rsp_ready),
    .param_bias_data(param_bias_data),.param_mult_data(param_mult_data),
    .param_shift_data(param_shift_data),.param_zero_point_data(param_zero_point_data),
    .result_rd_req_valid(latent_req),.result_rd_req_ready(latent_ready),
    .result_rd_addr0(latent_addr),.result_rd_addr1(latent_addr+32'd2),
    .result_rd_addr2(latent_addr+32'd4),.result_rd_addr3(latent_addr+32'd6),.result_rd_mask(4'hf),
    .result_rd_rsp_valid(latent_valid),.result_rd_rsp_ready(latent_rsp_ready),.result_rd_data(latent_data),
    .result_buffer_select(),.result_buffer_base(latent_base),
    .layer_write_fire(layer_write_fire),.layer_write_addr(layer_write_addr),
    .layer_write_data(layer_write_data),.layer_write_strb(layer_write_strb),
    .perf_engine_start(perf_engine_start),.perf_engine_done(perf_engine_done),
    .perf_weight_sram_read_fire(perf_weight_sram_read_fire),
    .perf_weight_prefetch_fire(perf_weight_prefetch_fire),
    .perf_weight_prefetch_done(perf_weight_prefetch_done),
    .perf_weight_prefetch_base(perf_weight_prefetch_base),
    .perf_weight_prefetch_blocks(perf_weight_prefetch_blocks),
    .flash_csn(flash_csn),.flash_sclk(flash_sclk),
    .flash_mosi(flash_mosi),.flash_miso(flash_miso),
    .flash_wp(flash_wpn),.flash_hold(flash_holdn),
    // 本Testbench使用片上快速Weight Cache模型，不启用Octal PHY。
    .octal_cmd_valid(octal_cmd_valid_unused),.octal_cmd_ready(1'b0),
    .octal_cmd_flash_base(octal_cmd_flash_base_unused),
    .octal_cmd_byte_count(octal_cmd_byte_count_unused),
    .octal_phy_clk(clk),.octal_phy_rst_n(rst_n),
    .octal_rx_valid(1'b0),.octal_rx_ready(octal_rx_ready_unused),
    .octal_rx_data(16'd0),.octal_rx_last(1'b0),.octal_rx_error(1'b0)
);

pcm16_input_frontend u_pcm_frontend (
    .clk(clk),.rst_n(rst_n),.start(pcm_frontend_start),
    .cfg_output_base(input_buffer_base),.cfg_sample_count(16'd320),
    .cfg_multiplier(pcm_multiplier_mem[0]),.cfg_shift(pcm_shift_mem[0][5:0]),.cfg_zero_point(8'sd0),
    .busy(pcm_frontend_busy),.done(pcm_frontend_done),
    .pcm_valid(pcm_valid),.pcm_ready(pcm_ready),.pcm_data(pcm_data),
    .wr_valid(input_wr_valid),.wr_ready(input_wr_ready),
    .wr_addr(input_wr_addr),.wr_data(input_wr_data),.wr_strb(input_wr_strb)
);

P25Q32H #(.Init_File("tb/data/flash_encoder_rvq/flash_image.hex")) u_flash (
    .CSb(flash_csn), .SCLK(flash_sclk), .SI(flash_mosi), .SO(flash_miso),
    .WPb(flash_wpn), .SIO3(flash_holdn)
);


always #(CLOCK_PERIOD_NS/2) clk = ~clk;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cycle_count <= 0;
    else
        cycle_count <= cycle_count + 1;
end

// 连续送入320个PCM样本；只在valid/ready握手时前进。
always @(*) begin
    pcm_valid = pcm_frontend_busy && (pcm_index < 320);
    pcm_data = (pcm_index < 320) ? pcm_memory[pcm_index] : 16'sd0;
end
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        pcm_index <= 0;
    else if (pcm_frontend_start && !pcm_frontend_busy)
        pcm_index <= 0;
    else if (pcm_valid && pcm_ready)
        pcm_index <= pcm_index + 1;
end

// 后处理参数由flash_boot_parameter_store同步读取，连接见support文件。
// 检查PCM量化及64-bit打包结果。
always @(posedge clk) begin
    if (rst_n && input_wr_valid && input_wr_ready) begin
        offset = (input_wr_addr - frame_input_base) >> 1;
        for (lane=0; lane<4; lane=lane+1) begin
            if (input_wr_strb[2*lane] || input_wr_strb[2*lane+1]) begin
                if (input_wr_data[16*lane +: 16] !== expected_pcm_activation[offset+lane]) begin
                    $display("[ERROR] PCM quant sample=%0d expected=%04h actual=%04h",
                             offset+lane,expected_pcm_activation[offset+lane],
                             input_wr_data[16*lane +: 16]);
                    errors = errors + 1;
                end
                pcm_compare_count = pcm_compare_count + 1;
            end
        end
    end
end

// 逐层、逐byte比较所有写回值。
always @(posedge clk) begin
    if (rst_n && layer_write_fire) begin
        // 63层为奇数：帧间输入Buffer自然交替，不能永远假设第0层从A读。
        expected_base = physical_layer[0] ? frame_input_base :
            ((frame_input_base == BUFFER_A_BASE) ? BUFFER_B_BASE : BUFFER_A_BASE);
        offset = layer_write_addr - expected_base;
        if ((offset < 0) || (offset >= 16384)) begin
            $display("[ERROR] layer=%0d output address out of range: %h",
                     physical_layer,layer_write_addr);
            errors = errors + 1;
        end else begin
            for (lane=0; lane<4; lane=lane+1) begin
                if (layer_write_strb[2*lane] || layer_write_strb[2*lane+1]) begin
                    golden_index = physical_layer * 8192 + (offset >> 1) + lane;
                    if (layer_write_data[16*lane +: 16] !== golden_layers[golden_index]) begin
                        if (errors < 50)
                            $display("[ERROR] layer=%0d element=%0d expected=%04h actual=%04h",
                                     physical_layer,(offset>>1)+lane,golden_layers[golden_index],
                                     layer_write_data[16*lane +: 16]);
                        errors = errors + 1;
                    end
                    actual_output_bytes[physical_layer] =
                        actual_output_bytes[physical_layer] + 2;
                    compare_count = compare_count + 1;
                end
            end
        end
    end
end

// 真实DUT事件的周期统计。
always @(posedge clk) begin
    if (rst_n) begin
        if (u_dut.u_encoder.first_weight_ready && (boot_weight_done_cycle < 0))
            boot_weight_done_cycle = cycle_count;

        if (perf_engine_start) begin
            $display("[PROGRESS] layer=%0d engine_start cycle=%0d",
                     physical_layer,cycle_count);
            compute_start_cycle[physical_layer] = cycle_count;
            expected_output_bytes[physical_layer] =
                expected_layer_elements[physical_layer] * 2;
            layer_start_errors[physical_layer] = errors;
        end
        if (perf_engine_done) begin
            compute_done_cycle[physical_layer] = cycle_count;
            // 普通卷积：每个256-bit权重块装载一次并在全部m位置复用。
            // Depthwise卷积：控制器会针对每个输出位置重新读取该层权重块，
            // 所以期望读取次数还需要乘以输出长度。
            expected_weight_reads = (physical_layer == 0) ? 4 :
                (u_dut.u_encoder.cfg_is_depthwise ?
                 (prefetch_blocks[physical_layer] * u_dut.u_encoder.cfg_output_length) :
                 prefetch_blocks[physical_layer]);
            $display("[PROGRESS] layer=%0d engine_done cycle=%0d comparisons=%0d",
                     physical_layer,cycle_count,compare_count);
            if ((errors == layer_start_errors[physical_layer]) &&
                (actual_output_bytes[physical_layer] ==
                 expected_output_bytes[physical_layer]) &&
                (weight_sram_read_count[physical_layer] == expected_weight_reads))
                $display("[LAYER_PASS] frame=%0d physical=%0d logical=%0d weight_reads=%0d bytes=%0d cycles=%0d",
                         frame_number,physical_layer,logical_layer,
                         weight_sram_read_count[physical_layer],
                         actual_output_bytes[physical_layer],
                         cycle_count-compute_start_cycle[physical_layer]);
            else begin
                $display("[LAYER_FAIL] frame=%0d physical=%0d logical=%0d expected_reads=%0d actual_reads=%0d expected_bytes=%0d actual_bytes=%0d new_errors=%0d",
                         frame_number,physical_layer,logical_layer,
                         expected_weight_reads,
                         weight_sram_read_count[physical_layer],
                         expected_output_bytes[physical_layer],
                         actual_output_bytes[physical_layer],
                         errors-layer_start_errors[physical_layer]);
                errors=errors+1;
            end
        end

        if (perf_weight_sram_read_fire) begin
            if (weight_sram_read_count[physical_layer] == 0)
                first_weight_read_cycle[physical_layer] = cycle_count;
            weight_sram_read_count[physical_layer] =
                weight_sram_read_count[physical_layer] + 1;
        end

        if (perf_weight_prefetch_fire) begin
            prefetch_pending_layer = physical_layer + 1;
            prefetch_start_cycle[prefetch_pending_layer] = cycle_count;
            prefetch_blocks[prefetch_pending_layer] = perf_weight_prefetch_blocks;
            $display("[PROGRESS] prefetch layer=%0d blocks=%0d start_cycle=%0d mode=%s",
                     prefetch_pending_layer,perf_weight_prefetch_blocks,cycle_count,
                     "GLOBAL_TO_LOCAL");
        end
        if (perf_weight_prefetch_done && (prefetch_pending_layer > 0)) begin
            prefetch_done_cycle[prefetch_pending_layer] = cycle_count;
            $display("[PROGRESS] prefetch layer=%0d done_cycle=%0d",
                     prefetch_pending_layer,cycle_count);
            prefetch_pending_layer = -1;
        end
    end
end


initial begin
    clk=0; rst_n=0; encoder_start=0; pcm_frontend_start=0;
    layer_prepare_ready=1;
    // 记录本次仿真的真实时钟配置，防止将旧频率日志误判为本次回归。
    $display("[CLOCK_CONFIG] system_hz=%0d qspi_hz=%0d period_ns=%0d",
             CLOCK_HZ, CLOCK_HZ / 2, CLOCK_PERIOD_NS);

    elu_lut_cfg_valid=0; elu_lut_cfg_addr=0; elu_lut_cfg_wdata=0;
    pcm_index=0; cycle_count=0; errors=0; compare_count=0; pcm_compare_count=0;
    frame_start_cycle=-1; encoder_start_cycle=-1; encoder_done_cycle=-1;
    boot_weight_done_cycle=-1; prefetch_pending_layer=-1;
    for (layer=0; layer<NUM_LAYERS; layer=layer+1) begin
        expected_output_bytes[layer]=0; actual_output_bytes[layer]=0;
        compute_start_cycle[layer]=-1; compute_done_cycle[layer]=-1;
        prefetch_start_cycle[layer]=-1; prefetch_done_cycle[layer]=-1;
        prefetch_blocks[layer]=0;
        layer_start_errors[layer]=0;
        first_weight_read_cycle[layer]=-1;
        weight_sram_read_count[layer]=0;
    end

    $readmemh("tb/data/w8a16_encoder/pcm16.hex", pcm_memory);
    $readmemh("tb/data/w8a16_encoder/pcm_activation_int16.hex", expected_pcm_activation);
    $readmemh("tb/data/w8a16_encoder/golden_layers.mem", golden_layers);
    $readmemh("tb/data/w8a16_encoder/expected_layer_elements.hex", expected_layer_elements);

    repeat (8) @(negedge clk);
    rst_n=1;
    @(negedge clk);
    load_boot_parameters;
    for (frame_number=0;frame_number<FRAME_COUNT;frame_number=frame_number+1) begin
    @(negedge clk);
    frame_errors=errors;
    if (FRAME_COUNT>1) load_independent_frame;
    frame_input_base=input_buffer_base;
    $display("[FRAME_START] frame=%0d input_base=%h cycle=%0d",frame_number,frame_input_base,cycle_count);
    frame_start_cycle=cycle_count;
    pcm_frontend_start=1;
    @(posedge clk);
    @(negedge clk);
    pcm_frontend_start=0;

    timeout_count=0;
    while (!pcm_frontend_done && timeout_count<10000) begin
        @(posedge clk);
        timeout_count=timeout_count+1;
    end
    if (!pcm_frontend_done) begin
        $display("[TB_FAIL] PCM frontend timeout");
        $finish;
    end
    if (pcm_compare_count != 320) begin
        $display("[ERROR] PCM comparisons expected=320 actual=%0d",pcm_compare_count);
        errors=errors+1;
    end

    // 当前W8A16网络使用ReLU；不启用INT8 ELU LUT。


    timeout_count=0;
    while (!encoder_start_ready && timeout_count<1000000) begin
        @(posedge clk);
        timeout_count=timeout_count+1;
    end
    if (!encoder_start_ready) begin
        $display("[TB_FAIL] first-layer weight boot timeout");
        $finish;
    end
    @(negedge clk);
    encoder_start_cycle=cycle_count;
    encoder_start=1;
    @(posedge clk);
    @(negedge clk);
    encoder_start=0;

    timeout_count=0;
    while (!encoder_done && timeout_count<MAX_TEST_CYCLES) begin
        @(posedge clk);
        timeout_count=timeout_count+1;
    end
    encoder_done_cycle=cycle_count;
    if (!encoder_done) begin
        $display("[TB_FAIL] full encoder timeout cycles=%0d",timeout_count);
        $finish;
    end

    for (layer=0; layer<NUM_LAYERS; layer=layer+1) begin
        if (actual_output_bytes[layer] != expected_output_bytes[layer]) begin
            $display("[ERROR] layer=%0d output count expected=%0d actual=%0d",
                     layer,expected_output_bytes[layer],actual_output_bytes[layer]);
            errors=errors+1;
        end
    end
    if (FRAME_COUNT>1) wait_multiframe_enqueued;
    else wait_rvq_complete;
    check_final_latent;
    print_flash_frame_report;

    if (encoder_error) begin
        $display("[ERROR] encoder_error asserted");
        errors=errors+1;
    end
    if (compare_count != EXPECTED_COMPARISONS) begin
        $display("[ERROR] activation comparisons expected=%0d actual=%0d",
                 EXPECTED_COMPARISONS,compare_count);
        errors=errors+1;
    end

    if (errors==frame_errors)
        $display("[FRAME_PASS] frame=%0d layers=63 stages=9 enqueued=1",frame_number);
    else $display("[FRAME_FAIL] frame=%0d new_errors=%0d",frame_number,errors-frame_errors);
    end
    if (FRAME_COUNT>1) finish_multiframe_dma;
    if (errors==0)
        $display("[TB_PASS] frames=%0d Flash + Encoder63 + Projection32 + RVQ9 + packed64 all checked; see TEST_MODE for parameter provenance",FRAME_COUNT);
    else
        $display("[TB_FAIL] errors=%0d comparisons=%0d",errors,compare_count);
    $finish;
end

`undef HIER
endmodule
