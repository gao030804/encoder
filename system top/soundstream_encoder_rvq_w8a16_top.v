module soundstream_encoder_rvq_w8a16_top #(
    parameter ADDR_WIDTH = 32,
    parameter BUFFER_A_BASE = 32'h0000_1000,
    parameter BUFFER_B_BASE = 32'h0000_5000,

    // 保留旧参数名以兼容现有testbench；Global路径中含义为QSPI半周期分频值。
    // QSPI_HALF_DIV=1时，Flash SCLK为主时钟的1/2；50 MHz主时钟得到25 MHz。
    parameter FLASH_SPI_DIVIDER = 16'd1,

    // 置1后采用Flash->256 KiB Global->2x8 KiB Local的Encoder权重层级。
    // FAST_SIM_WEIGHT_CACHE仍具有更高优先级，便于已有全帧testbench快速运行。
    // 当前63层ROM使用162688 Byte packed weight，默认256 KiB Global SRAM可完整容纳。
    // 使用最新紧凑Encoder权重镜像和层描述表时将该参数置1。
    parameter USE_GLOBAL_WEIGHT_HIERARCHY = 1,
    parameter GLOBAL_DEPTH_BLOCKS = 8192,
    parameter GLOBAL_INDEX_WIDTH = 13,
    parameter GLOBAL_MODEL_BLOCKS = 6912,
    parameter BOOT_AUX_BLOCKS = 1536,
    parameter BOOT_FLASH_BASE = 24'h010000,
    parameter USE_OCTAL_STREAM_WEIGHT_CACHE = 0,
    parameter FAST_SIM_WEIGHT_CACHE = 0,
    parameter RESIDUAL_SCALE_ALIGN_ENABLE = 1,
    parameter CLOCK_HZ = 50000000,
    // 正常部署只输出索引，32维重建向量交给软件；置1仅用于对拍调试。
    parameter ENABLE_VECTOR_DEBUG = 0
) (
    output encoder_done,
    // encoded_valid/indices是DMA队首的兼容观察口，消费握手统一使用dma_ready。
    output encoded_valid,
    output [63:0] encoded_indices,
    output [511:0] quantized_vector,
    output dma_valid, input dma_ready,
    output [127:0] dma_data,
    output [15:0] dma_keep,
    output dma_last,
    output [8:0] output_level,
    output output_full, output output_almost_full, output output_stall_seen,
    output boot_aux_write,
    output [15:0] boot_aux_index,
    output [255:0] boot_aux_data,
    output boot_param_write,
    output [12:0] boot_param_block,
    output [255:0] boot_param_data,
    output boot_ready,
    input                                       clk,
    input                                       rst_n,
    // 芯片集成必须明确驱动：功能模式为0，DFT扫描模式可置1旁路时钟门控。
    input wire                                  mesh_scan_test_en,
    input                                       start,
    output                                      start_ready,
    output                                      busy,
    output                                      done,
    output                                      error,

    // 编码前由PCM frontend向当前输入Buffer写入320个INT16激活。
    input                                       input_wr_valid,
    output                                      input_wr_ready,
    input              [ADDR_WIDTH-1:0]         input_wr_addr,
    input              [63:0]                   input_wr_data,
    input              [7:0]                    input_wr_strb,
    output                                      input_buffer_select,
    output             [ADDR_WIDTH-1:0]         input_buffer_base,

    // 每层开始前的准备握手；外部可在此阶段更新该层ELU LUT。
    output                                      layer_prepare_valid,
    input                                       layer_prepare_ready,
    // 与layer_prepare握手同步提交，当前层执行期间保持在内部寄存器中。
    input              [119:0]                  residual_scale_params,
    output             [5:0]                    physical_layer,
    output             [5:0]                    logical_layer,
    input                                       elu_lut_cfg_valid,
    output                                      elu_lut_cfg_ready,
    input              [6:0]                    elu_lut_cfg_addr,
    input              [7:0]                    elu_lut_cfg_wdata,

    // Parameter Memory用physical_layer和output_group联合寻址。
    output                                      param_rd_req_valid,
    input                                       param_rd_req_ready,
    output             [5:0]                    param_rd_layer,
    output             [5:0]                    param_rd_output_group,
    input                                       param_rd_rsp_valid,
    output                                      param_rd_rsp_ready,
    input              [511:0]                  param_bias_data,
    input              [255:0]                  param_mult_data,
    input              [63:0]                   param_shift_data,
    input              [63:0]                   param_zero_point_data,

    // RVQ占用读口时外部访问反压；后端结束后可读回1x64 INT16 latent。
    input                                       result_rd_req_valid,
    output                                      result_rd_req_ready,
    input              [ADDR_WIDTH-1:0]         result_rd_addr0,
    input              [ADDR_WIDTH-1:0]         result_rd_addr1,
    input              [ADDR_WIDTH-1:0]         result_rd_addr2,
    input              [ADDR_WIDTH-1:0]         result_rd_addr3,
    input              [3:0]                    result_rd_mask,
    output                                      result_rd_rsp_valid,
    input                                       result_rd_rsp_ready,
    output             [63:0]                   result_rd_data,
    output                                      result_buffer_select,
    output             [ADDR_WIDTH-1:0]         result_buffer_base,

    // 逐层写回监视口，不参与反压，供验证和性能统计。
    output                                      layer_write_fire,
    output             [ADDR_WIDTH-1:0]         layer_write_addr,
    output             [63:0]                   layer_write_data,
    output             [7:0]                    layer_write_strb,

    // 性能计数观测口：不参与控制，仅供testbench/片上性能计数器使用。
    output                                      perf_engine_start,
    output                                      perf_engine_done,
    output                                      perf_weight_sram_read_fire,
    output                                      perf_weight_prefetch_fire,
    output                                      perf_weight_prefetch_done,
    output             [23:0]                   perf_weight_prefetch_base,
    output             [15:0]                   perf_weight_prefetch_blocks,
    output                                      flash_csn,
    output                                      flash_sclk,
    inout                                       flash_mosi,
    inout                                       flash_miso,
    inout                                       flash_wp,
    inout                                       flash_hold,

    // S28 OPI控制器/PHY完成DDR采样和CDC之后的命令/16-bit数据流接口。
    output                                      octal_cmd_valid,
    input                                       octal_cmd_ready,
    output             [23:0]                   octal_cmd_flash_base,
    output             [23:0]                   octal_cmd_byte_count,
    input                                       octal_phy_clk,
    input                                       octal_phy_rst_n,
    input                                       octal_rx_valid,
    output                                      octal_rx_ready,
    input              [15:0]                   octal_rx_data,
    input                                       octal_rx_last,
    input                                       octal_rx_error
);
// 自动启动后端，编码完成后通过原结果SRAM端口读取64维latent。
// done表示码流已写入环形SRAM；DMA读走可发生在后续任意时刻。
wire enc_ready,enc_busy,enc_error,backend_busy,backend_error;
wire backend_configured,encoder_input_ready;
wire back_req,back_rsp_ready,rd_ready,rd_valid;
wire [31:0] back_addr;
wire [63:0] rd_data;
wire backend_result_valid,backend_result_ready;
wire [63:0] backend_indices;
reg [31:0] frame_id;
reg [31:0] timestamp_us, timestamp_divider, frame_timestamp_us;
localparam integer US_CYCLES = CLOCK_HZ/1000000;
// 时间戳是本帧start握手时刻，单位us，复位清零，约71.6分钟回绕。
// 外部实时采集系统若需要采样时刻，应在上层用相同frame_id关联ADC时间戳。
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        frame_id<=0;timestamp_us<=0;timestamp_divider<=0;frame_timestamp_us<=0;
    end else begin
        if(timestamp_divider==US_CYCLES-1) begin
            timestamp_divider<=0;timestamp_us<=timestamp_us+1'b1;
        end else timestamp_divider<=timestamp_divider+1'b1;
        if(start && start_ready) frame_timestamp_us<=timestamp_us;
        if(backend_result_valid && backend_result_ready) frame_id<=frame_id+1'b1;
    end
end
assign start_ready=enc_ready && !backend_busy && !encoder_done && backend_configured && !output_full;
assign input_wr_ready=encoder_input_ready && !backend_busy && !encoder_done;
assign busy=enc_busy || backend_busy;
assign done=backend_result_valid && backend_result_ready;
assign encoded_valid=dma_valid;
assign encoded_indices=dma_data[63:0];
assign error=enc_error || backend_error;
assign result_rd_req_ready=!backend_busy && rd_ready;
assign result_rd_rsp_valid=!backend_busy && rd_valid;
assign result_rd_data=rd_data;
soundstream_encoder_top #(
    .ADDR_WIDTH(ADDR_WIDTH),
    .BUFFER_A_BASE(BUFFER_A_BASE),
    .BUFFER_B_BASE(BUFFER_B_BASE),
    .FLASH_SPI_DIVIDER(FLASH_SPI_DIVIDER),
    .USE_GLOBAL_WEIGHT_HIERARCHY(USE_GLOBAL_WEIGHT_HIERARCHY),
    .GLOBAL_DEPTH_BLOCKS(GLOBAL_DEPTH_BLOCKS),
    .GLOBAL_INDEX_WIDTH(GLOBAL_INDEX_WIDTH),
    .GLOBAL_MODEL_BLOCKS(GLOBAL_MODEL_BLOCKS),
    .BOOT_AUX_BLOCKS(BOOT_AUX_BLOCKS),
    .BOOT_FLASH_BASE(BOOT_FLASH_BASE),
    .USE_OCTAL_STREAM_WEIGHT_CACHE(USE_OCTAL_STREAM_WEIGHT_CACHE),
    .FAST_SIM_WEIGHT_CACHE(FAST_SIM_WEIGHT_CACHE),
    .RESIDUAL_SCALE_ALIGN_ENABLE(RESIDUAL_SCALE_ALIGN_ENABLE)
) u_encoder (
    .mesh_scan_test_en(mesh_scan_test_en),
    .boot_aux_write(boot_aux_write),
    .boot_aux_index(boot_aux_index),
    .boot_aux_data(boot_aux_data),
    .boot_param_write(boot_param_write),
    .boot_param_block(boot_param_block),
    .boot_param_data(boot_param_data),
    .boot_ready(boot_ready),
    .clk(clk),
    .rst_n(rst_n),
    .start(start && start_ready),
    .start_ready(enc_ready),
    .busy(enc_busy),
    .done(encoder_done),
    .error(enc_error),
    .input_wr_valid(input_wr_valid && !backend_busy && !encoder_done),
    .input_wr_ready(encoder_input_ready),
    .input_wr_addr(input_wr_addr),
    .input_wr_data(input_wr_data),
    .input_wr_strb(input_wr_strb),
    .input_buffer_select(input_buffer_select),
    .input_buffer_base(input_buffer_base),
    .layer_prepare_valid(layer_prepare_valid),
    .layer_prepare_ready(layer_prepare_ready),
    .residual_scale_params(residual_scale_params),
    .physical_layer(physical_layer),
    .logical_layer(logical_layer),
    .elu_lut_cfg_valid(elu_lut_cfg_valid),
    .elu_lut_cfg_ready(elu_lut_cfg_ready),
    .elu_lut_cfg_addr(elu_lut_cfg_addr),
    .elu_lut_cfg_wdata(elu_lut_cfg_wdata),
    .param_rd_req_valid(param_rd_req_valid),
    .param_rd_req_ready(param_rd_req_ready),
    .param_rd_layer(param_rd_layer),
    .param_rd_output_group(param_rd_output_group),
    .param_rd_rsp_valid(param_rd_rsp_valid),
    .param_rd_rsp_ready(param_rd_rsp_ready),
    .param_bias_data(param_bias_data),
    .param_mult_data(param_mult_data),
    .param_shift_data(param_shift_data),
    .param_zero_point_data(param_zero_point_data),
    .result_rd_req_valid(backend_busy ? back_req : result_rd_req_valid),
    .result_rd_req_ready(rd_ready),
    .result_rd_addr0(backend_busy ? back_addr : result_rd_addr0),
    .result_rd_addr1(backend_busy ? back_addr+32'd2 : result_rd_addr1),
    .result_rd_addr2(backend_busy ? back_addr+32'd4 : result_rd_addr2),
    .result_rd_addr3(backend_busy ? back_addr+32'd6 : result_rd_addr3),
    .result_rd_mask(backend_busy ? 4'hf : result_rd_mask),
    .result_rd_rsp_valid(rd_valid),
    .result_rd_rsp_ready(backend_busy ? back_rsp_ready : result_rd_rsp_ready),
    .result_rd_data(rd_data),
    .result_buffer_select(result_buffer_select),
    .result_buffer_base(result_buffer_base),
    .layer_write_fire(layer_write_fire),
    .layer_write_addr(layer_write_addr),
    .layer_write_data(layer_write_data),
    .layer_write_strb(layer_write_strb),
    .perf_engine_start(perf_engine_start),
    .perf_engine_done(perf_engine_done),
    .perf_weight_sram_read_fire(perf_weight_sram_read_fire),
    .perf_weight_prefetch_fire(perf_weight_prefetch_fire),
    .perf_weight_prefetch_done(perf_weight_prefetch_done),
    .perf_weight_prefetch_base(perf_weight_prefetch_base),
    .perf_weight_prefetch_blocks(perf_weight_prefetch_blocks),
    .flash_csn(flash_csn),
    .flash_sclk(flash_sclk),
    .flash_mosi(flash_mosi),
    .flash_miso(flash_miso),
    .flash_wp(flash_wp),
    .flash_hold(flash_hold),
    .octal_cmd_valid(octal_cmd_valid),
    .octal_cmd_ready(octal_cmd_ready),
    .octal_cmd_flash_base(octal_cmd_flash_base),
    .octal_cmd_byte_count(octal_cmd_byte_count),
    .octal_phy_clk(octal_phy_clk),
    .octal_phy_rst_n(octal_phy_rst_n),
    .octal_rx_valid(octal_rx_valid),
    .octal_rx_ready(octal_rx_ready),
    .octal_rx_data(octal_rx_data),
    .octal_rx_last(octal_rx_last),
    .octal_rx_error(octal_rx_error)
);
rvq_backend_w8a16 #(.COMPUTE_VECTOR(ENABLE_VECTOR_DEBUG)) u_backend (
    .clk(clk),.rst_n(rst_n),.boot_write(boot_aux_write),.boot_index(boot_aux_index),.boot_data(boot_aux_data),
    .start(encoder_done),.busy(backend_busy),.error(backend_error),.configured(backend_configured),
    .latent_base(result_buffer_base),.latent_req_valid(back_req),.latent_req_ready(rd_ready),
    .latent_addr(back_addr),.latent_rsp_valid(rd_valid),.latent_rsp_ready(back_rsp_ready),.latent_data(rd_data),
    .result_valid(backend_result_valid),.result_ready(backend_result_ready),
    .result_indices(backend_indices),.result_vector(quantized_vector),
    // 以下端口只用于内部逐级调试；板级数据通路不消费这些观测信号。
    .projection_valid(),.projection_channel(),.projection_data(),
    .score_valid(),.trace_stage(),.trace_candidate(),.trace_score(),
    .update_valid(),.trace_dimension(),.trace_residual(),
    .trace_codeword(),.trace_subtract(),.trace_next(),
    .stage_valid(),.trace_index()
);
encoded_output_ring u_output_ring (
    .clk(clk),.rst_n(rst_n),.in_valid(backend_result_valid),.in_ready(backend_result_ready),
    .in_indices(backend_indices),.in_frame_id(frame_id),.in_timestamp_us(frame_timestamp_us),
    .dma_valid(dma_valid),.dma_ready(dma_ready),.dma_data(dma_data),.dma_keep(dma_keep),.dma_last(dma_last),
    .level(output_level),.full(output_full),.almost_full(output_almost_full),.stall_seen(output_stall_seen)
);
endmodule
