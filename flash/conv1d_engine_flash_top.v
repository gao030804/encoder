// ============================================================================
// 中文阅读导引（当前实现）
// 单层卷积与权重存储的连接层。内部u_engine负责计算，generate选择实际权重来源。
// 选择优先级：FAST_SIM_WEIGHT_CACHE → USE_GLOBAL_WEIGHT_HIERARCHY → Octal接口 → 原SPI缓存。
// FAST_SIM分支仅供仿真；Octal分支的PHY数据由外部端口提供。
// ============================================================================
//=============================================================================
// 模块：conv1d_engine_flash_top
// 功能：Flash权重版本的Conv1d NPU顶层。
//
// 与conv1d_engine_top相比，本顶层不再把256-bit Weight SRAM接口引出芯片，
// 而是内部实例化flash_weight_reader，通过4根单线SPI信号访问片外W25Q128。
// 原conv1d_engine_top仍然保留，便于SRAM模型仿真和模块级验证。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - 计算与权重存储封装层。内部计算引擎始终看到统一256-bit Weight SRAM读接口。
// - generate优先级为快速仿真模型、Global/Local层级、Octal路径、旧SPI A/B Cache。
// - 综合配置只能启用一种权重来源；不同分支的未使用外部端口会被固定到安全值。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：卷积引擎与权重层次封装。
// 关键变量/接口：根据参数选择快速模型、Global/Local、Octal或SPI缓存，并连接统一Weight SRAM接口。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module conv1d_engine_flash_top #(
    parameter ADDR_WIDTH = 32,
    parameter M_TAG_WIDTH = 9,
    parameter MAX_M = 320,
    parameter FLASH_SPI_DIVIDER = 16'd1,
    parameter WEIGHT_CACHE_BLOCKS = 4096,
    parameter WEIGHT_CACHE_INDEX_WIDTH = 12,
    parameter FIRST_LAYER_FLASH_BASE = 24'h000000,

    // 当前63层方案：Flash上电一次装入256 KiB Global SRAM，再按Tile搬到8 KiB Local A/B。
    parameter USE_GLOBAL_WEIGHT_HIERARCHY = 0,
    parameter GLOBAL_DEPTH_BLOCKS = 8192,
    parameter GLOBAL_INDEX_WIDTH = 13,
    parameter GLOBAL_MODEL_BLOCKS = 5084,
    parameter BOOT_AUX_BLOCKS = 0,
    parameter BOOT_FLASH_BASE = FIRST_LAYER_FLASH_BASE,

    // 置1时使用Octal控制器16-bit接收流；置0时保留原单线SPI路径。
    parameter USE_OCTAL_STREAM_CACHE = 0,

    // 仅完整帧仿真置1；综合和真实SPI测试必须保持0。
    parameter FAST_SIM_WEIGHT_CACHE = 0
) (
    output boot_aux_write,
    output [15:0] boot_aux_index,
    output [255:0] boot_aux_data,
    output boot_param_write,
    output [12:0] boot_param_block,
    output [255:0] boot_param_data,
    input                                       clk,
    input                                       rst_n,
    // 普通wire端口供ASIC综合；功能模式显式接0，禁止依赖悬空默认值。
    input wire                                  mesh_scan_test_en,
    input                                       start,
    output                                      busy,
    output                                      done,
    output                                      error,
    output                                      flash_busy,
    output                                      flash_error,
    output                                      first_layer_weight_ready,
    output                                      active_weight_is_first_layer,
    output                                      active_weight_bank,
    output                                      active_weight_bank_valid,

    // 从片上Weight SRAM接受一个256-bit读请求的性能观测脉冲。
    output                                      perf_weight_read_fire,

    // 后续层预取到非活动Bank；一层计算期间即可启动下一层预取。
    input                                       weight_prefetch_valid,
    output                                      weight_prefetch_ready,
    input              [23:0]                   weight_prefetch_flash_base,
    input              [15:0]                   weight_prefetch_block_count,
    input                                       weight_prefetch_target_bank,
    output                                      weight_prefetch_done,

    // 当前层结束、下一层开始前显式切换活动Bank。
    input                                       weight_activate_valid,
    output                                      weight_activate_ready,
    input                                       weight_activate_first_layer,
    input                                       weight_activate_bank,
    input              [31:0]                   cfg_input_base,
    input              [31:0]                   cfg_output_base,

    // 片外Flash中的byte地址，W25Q128有效范围为0x000000～0xffffff。
    input              [31:0]                   cfg_weight_base,
    input              [8:0]                    cfg_cin,
    input              [8:0]                    cfg_cout,
    input              [4:0]                    cfg_kernel,
    input              [3:0]                    cfg_stride,
    input              [3:0]                    cfg_dilation,
    input              [7:0]                    cfg_left_pad,
    input              [8:0]                    cfg_input_length,
    input              [8:0]                    cfg_output_length,
    input              [11:0]                   cfg_k_total,
    input              [9:0]                    cfg_k_groups,
    input              [5:0]                    cfg_n_groups,
    input                                       cfg_is_depthwise,
    input                                       cfg_relu_enable,
    input                                       cfg_elu_enable,
    input                                       elu_lut_cfg_valid,
    output                                      elu_lut_cfg_ready,
    input              [6:0]                    elu_lut_cfg_addr,
    input              [7:0]                    elu_lut_cfg_wdata,
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
    output                                      param_rd_req_valid,
    input                                       param_rd_req_ready,
    output             [5:0]                    param_rd_output_group,
    input                                       param_rd_rsp_valid,
    output                                      param_rd_rsp_ready,
    input              [511:0]                  param_bias_data,
    input              [255:0]                  param_mult_data,
    input              [63:0]                   param_shift_data,
    input              [63:0]                   param_zero_point_data,
    output                                      out_wr_valid,
    input                                       out_wr_ready,
    output             [ADDR_WIDTH-1:0]         out_wr_addr,
    output             [63:0]                   out_wr_data,
    output             [7:0]                    out_wr_strb,
    output                                      flash_csn,
    output                                      flash_sclk,
    inout                                       flash_mosi,
    inout                                       flash_miso,
    inout                                       flash_wp,
    inout                                       flash_hold,

    // S28 OPI控制器/PHY边界。PHY负责引脚DDR采样与CDC，本模块负责DMA和SRAM。
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

wire                      engine_error;
wire                      wgt_rd_req_valid;
wire                      wgt_rd_req_ready;
wire [ADDR_WIDTH-1:0]     wgt_rd_addr;
wire                      wgt_rd_rsp_valid;
wire                      wgt_rd_rsp_ready;
wire [255:0]              wgt_rd_data;
// 连续赋值：组合生成error及其相邻接口信号，表达握手、选择或地址关系。
assign error = engine_error | flash_error;
assign perf_weight_read_fire = wgt_rd_req_valid && wgt_rd_req_ready;
conv1d_engine_top #(
    .ADDR_WIDTH                  (ADDR_WIDTH),
    .M_TAG_WIDTH                 (M_TAG_WIDTH),
    .MAX_M                       (MAX_M)
) u_engine (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .mesh_scan_test_en            (mesh_scan_test_en),
    .start                       (start),
    .busy                        (busy),
    .done                        (done),
    .error                       (engine_error),
    .cfg_input_base              (cfg_input_base),
    .cfg_output_base             (cfg_output_base),
    .cfg_weight_base             (cfg_weight_base),
    .cfg_cin                     (cfg_cin),
    .cfg_cout                    (cfg_cout),
    .cfg_kernel                  (cfg_kernel),
    .cfg_stride                  (cfg_stride),
    .cfg_dilation                (cfg_dilation),
    .cfg_left_pad                (cfg_left_pad),
    .cfg_input_length            (cfg_input_length),
    .cfg_output_length           (cfg_output_length),
    .cfg_k_total                 (cfg_k_total),
    .cfg_k_groups                (cfg_k_groups),
    .cfg_n_groups                (cfg_n_groups),
    .cfg_is_depthwise            (cfg_is_depthwise),
    .cfg_relu_enable             (cfg_relu_enable),
    .cfg_elu_enable              (cfg_elu_enable),
    .elu_lut_cfg_valid           (elu_lut_cfg_valid),
    .elu_lut_cfg_ready           (elu_lut_cfg_ready),
    .elu_lut_cfg_addr            (elu_lut_cfg_addr),
    .elu_lut_cfg_wdata           (elu_lut_cfg_wdata),
    .act_rd_req_valid            (act_rd_req_valid),
    .act_rd_req_ready            (act_rd_req_ready),
    .act_rd_addr0                (act_rd_addr0),
    .act_rd_addr1                (act_rd_addr1),
    .act_rd_addr2                (act_rd_addr2),
    .act_rd_addr3                (act_rd_addr3),
    .act_rd_mask                 (act_rd_mask),
    .act_rd_rsp_valid            (act_rd_rsp_valid),
    .act_rd_rsp_ready            (act_rd_rsp_ready),
    .act_rd_data                 (act_rd_data),
    .wgt_rd_req_valid            (wgt_rd_req_valid),
    .wgt_rd_req_ready            (wgt_rd_req_ready),
    .wgt_rd_addr                 (wgt_rd_addr),
    .wgt_rd_rsp_valid            (wgt_rd_rsp_valid),
    .wgt_rd_rsp_ready            (wgt_rd_rsp_ready),
    .wgt_rd_data                 (wgt_rd_data),
    .param_rd_req_valid          (param_rd_req_valid),
    .param_rd_req_ready          (param_rd_req_ready),
    .param_rd_output_group       (param_rd_output_group),
    .param_rd_rsp_valid          (param_rd_rsp_valid),
    .param_rd_rsp_ready          (param_rd_rsp_ready),
    .param_bias_data             (param_bias_data),
    .param_mult_data             (param_mult_data),
    .param_shift_data            (param_shift_data),
    .param_zero_point_data       (param_zero_point_data),
    .out_wr_valid                (out_wr_valid),
    .out_wr_ready                (out_wr_ready),
    .out_wr_addr                 (out_wr_addr),
    .out_wr_data                 (out_wr_data),
    .out_wr_strb                 (out_wr_strb)
);

// 参数化生成块：综合时只保留满足参数条件的硬件分支，未选分支不参与数据通路。
generate
    // 旧缓存模式不产生RVQ启动数据流，显式输出0，避免悬空信号。
    if (FAST_SIM_WEIGHT_CACHE || !USE_GLOBAL_WEIGHT_HIERARCHY) begin : g_no_aux_boot
        assign boot_aux_write = 1'b0;
        assign boot_aux_index = 16'd0;
        assign boot_aux_data = 256'd0;
        assign boot_param_write = 1'b0;
        assign boot_param_block = 13'd0;
        assign boot_param_data = 256'd0;
    end
    if (FAST_SIM_WEIGHT_CACHE) begin : g_fast_sim_weight_cache
            // 连续赋值：组合生成octal_cmd_valid及其相邻接口信号，表达握手、选择或地址关系。
            assign octal_cmd_valid      = 1'b0;
        assign octal_cmd_flash_base = 24'd0;
        // 连续赋值：组合生成octal_cmd_byte_count及其相邻接口信号，表达握手、选择或地址关系。
        assign octal_cmd_byte_count = 24'd0;
        assign octal_rx_ready       = 1'b0;
        weight_cache_ab_fast_model u_weight_cache (
            .clk                         (clk),
            .rst_n                       (rst_n),
            .req_valid                   (wgt_rd_req_valid),
            .req_ready                   (wgt_rd_req_ready),
            .req_addr                    (wgt_rd_addr),
            .rsp_valid                   (wgt_rd_rsp_valid),
            .rsp_ready                   (wgt_rd_rsp_ready),
            .rsp_data                    (wgt_rd_data),
            .prefetch_valid              (weight_prefetch_valid),
            .prefetch_ready              (weight_prefetch_ready),
            .prefetch_flash_base         (weight_prefetch_flash_base),
            .prefetch_block_count        (weight_prefetch_block_count),
            .prefetch_target_bank        (weight_prefetch_target_bank),
            .prefetch_done               (weight_prefetch_done),
            .activate_valid              (weight_activate_valid),
            .activate_ready              (weight_activate_ready),
            .activate_first_layer        (weight_activate_first_layer),
            .activate_bank               (weight_activate_bank),
            .active_first_layer          (active_weight_is_first_layer),
            .active_bank                 (active_weight_bank),
            .active_bank_valid           (active_weight_bank_valid),
            .first_layer_ready           (first_layer_weight_ready),
            .cache_busy                  (flash_busy),
            .cache_error                 (flash_error),
            .flash_csn                   (flash_csn),
            .flash_sclk                  (flash_sclk),
            .flash_mosi                  (flash_mosi),
            .flash_miso                  (flash_miso)
        );

        // 连续赋值：组合生成flash_wp及其相邻接口信号，表达握手、选择或地址关系。
        assign flash_wp   = 1'bz;
        assign flash_hold = 1'bz;
    end else if (USE_GLOBAL_WEIGHT_HIERARCHY) begin : g_encoder_weight_hierarchy
        // 连续赋值：组合生成octal_cmd_valid及其相邻接口信号，表达握手、选择或地址关系。
        assign octal_cmd_valid      = 1'b0;
        assign octal_cmd_flash_base = 24'd0;
        // 连续赋值：组合生成octal_cmd_byte_count及其相邻接口信号，表达握手、选择或地址关系。
        assign octal_cmd_byte_count = 24'd0;
        assign octal_rx_ready       = 1'b0;
        wire global_sram_active_unused;
        wire local_sram_a_active_unused;
        wire local_sram_b_active_unused;
        encoder_weight_hierarchy #(
            .GLOBAL_DEPTH_BLOCKS         (GLOBAL_DEPTH_BLOCKS),
            .GLOBAL_INDEX_WIDTH          (GLOBAL_INDEX_WIDTH),
            .GLOBAL_MODEL_BLOCKS         (GLOBAL_MODEL_BLOCKS),
            .BOOT_AUX_BLOCKS             (BOOT_AUX_BLOCKS),
            .BOOT_FLASH_BASE             (BOOT_FLASH_BASE),
            .LOCAL_DEPTH_BLOCKS          (256),
            .LOCAL_INDEX_WIDTH           (8),
            .FIRST_LAYER_FLASH_BASE      (FIRST_LAYER_FLASH_BASE),
            .FIRST_LAYER_BLOCKS          (4),
            .QSPI_HALF_DIV               (FLASH_SPI_DIVIDER),
            .FLASH_POWER_UP_CYCLES       (32'd7000),
            .QSPI_DUMMY_CYCLES           (8)
        ) u_weight_hierarchy (
            .boot_aux_write(boot_aux_write),.boot_aux_index(boot_aux_index),.boot_aux_data(boot_aux_data),
            .boot_param_write(boot_param_write),.boot_param_block(boot_param_block),
            .boot_param_data(boot_param_data),
            .clk                         (clk),
            .rst_n                       (rst_n),
            .req_valid                   (wgt_rd_req_valid),
            .req_ready                   (wgt_rd_req_ready),
            .req_addr                    (wgt_rd_addr),
            .rsp_valid                   (wgt_rd_rsp_valid),
            .rsp_ready                   (wgt_rd_rsp_ready),
            .rsp_data                    (wgt_rd_data),
            .prefetch_valid              (weight_prefetch_valid),
            .prefetch_ready              (weight_prefetch_ready),
            .prefetch_global_base        (weight_prefetch_flash_base),
            .prefetch_block_count        (weight_prefetch_block_count),
            .prefetch_target_bank        (weight_prefetch_target_bank),
            .prefetch_done               (weight_prefetch_done),
            .activate_valid              (weight_activate_valid),
            .activate_ready              (weight_activate_ready),
            .activate_first_layer        (weight_activate_first_layer),
            .activate_bank               (weight_activate_bank),
            .active_first_layer          (active_weight_is_first_layer),
            .active_bank                 (active_weight_bank),
            .active_bank_valid           (active_weight_bank_valid),
            .first_layer_ready           (first_layer_weight_ready),
            .hierarchy_busy              (flash_busy),
            .hierarchy_error             (flash_error),
            .global_sram_active          (global_sram_active_unused),
            .local_sram_a_active         (local_sram_a_active_unused),
            .local_sram_b_active         (local_sram_b_active_unused),
            .flash_csn                   (flash_csn),
            .flash_sclk                  (flash_sclk),
            .flash_mosi                  (flash_mosi),
            .flash_miso                  (flash_miso),
            .flash_wp                    (flash_wp),
            .flash_hold                  (flash_hold)
        );

    end else if (USE_OCTAL_STREAM_CACHE) begin : g_octal_weight_cache
        // 连续赋值：组合生成flash_csn及其相邻接口信号，表达握手、选择或地址关系。
        assign flash_csn  = 1'b1;
        assign flash_sclk = 1'b0;
        // 连续赋值：组合生成flash_mosi及其相邻接口信号，表达握手、选择或地址关系。
        assign flash_mosi = 1'b0;
        assign flash_wp   = 1'bz;
        // 连续赋值：组合生成flash_hold及其相邻接口信号，表达握手、选择或地址关系。
        assign flash_hold = 1'bz;
        wire        sys_rx_valid;
        wire        sys_rx_ready;
        wire [17:0] sys_rx_payload;

        // PHY域每拍16 bit；异步FIFO完成CDC，随后在系统域进行16→256 bit组包。
        octal_ddr_async_fifo #(
            .DATA_WIDTH                  (18),
            .ADDR_WIDTH                  (5)
        ) u_octal_rx_cdc_fifo (
            .wr_clk                      (octal_phy_clk),
            .wr_rst_n                    (octal_phy_rst_n),
            .wr_valid                    (octal_rx_valid),
            .wr_ready                    (octal_rx_ready),
            .wr_data                     ({octal_rx_error,octal_rx_last,octal_rx_data}),
            .rd_clk                      (clk),
            .rd_rst_n                    (rst_n),
            .rd_valid                    (sys_rx_valid),
            .rd_ready                    (sys_rx_ready),
            .rd_data                     (sys_rx_payload)
        );

        weight_cache_ab_octal #(
            .CACHE_BLOCKS                (WEIGHT_CACHE_BLOCKS),
            .CACHE_INDEX_WIDTH           (WEIGHT_CACHE_INDEX_WIDTH),
            .FIRST_LAYER_FLASH_BASE      (FIRST_LAYER_FLASH_BASE),
            .FIRST_LAYER_BLOCKS          (16'd4)
        ) u_weight_cache (
            .clk                         (clk),
            .rst_n                       (rst_n),
            .req_valid                   (wgt_rd_req_valid),
            .req_ready                   (wgt_rd_req_ready),
            .req_addr                    (wgt_rd_addr),
            .rsp_valid                   (wgt_rd_rsp_valid),
            .rsp_ready                   (wgt_rd_rsp_ready),
            .rsp_data                    (wgt_rd_data),
            .prefetch_valid              (weight_prefetch_valid),
            .prefetch_ready              (weight_prefetch_ready),
            .prefetch_flash_base         (weight_prefetch_flash_base),
            .prefetch_block_count        (weight_prefetch_block_count),
            .prefetch_target_bank        (weight_prefetch_target_bank),
            .prefetch_done               (weight_prefetch_done),
            .activate_valid              (weight_activate_valid),
            .activate_ready              (weight_activate_ready),
            .activate_first_layer        (weight_activate_first_layer),
            .activate_bank               (weight_activate_bank),
            .active_first_layer          (active_weight_is_first_layer),
            .active_bank                 (active_weight_bank),
            .active_bank_valid           (active_weight_bank_valid),
            .first_layer_ready           (first_layer_weight_ready),
            .cache_busy                  (flash_busy),
            .cache_error                 (flash_error),
            .octal_cmd_valid             (octal_cmd_valid),
            .octal_cmd_ready             (octal_cmd_ready),
            .octal_cmd_flash_base        (octal_cmd_flash_base),
            .octal_cmd_byte_count        (octal_cmd_byte_count),
            .octal_rx_valid              (sys_rx_valid),
            .octal_rx_ready              (sys_rx_ready),
            .octal_rx_data               (sys_rx_payload[15:0]),
            .octal_rx_last               (sys_rx_payload[16]),
            .octal_rx_error              (sys_rx_payload[17])
        );

    end else begin : g_real_weight_cache
        // 连续赋值：组合生成octal_cmd_valid及其相邻接口信号，表达握手、选择或地址关系。
        assign octal_cmd_valid      = 1'b0;
        assign octal_cmd_flash_base = 24'd0;
        // 连续赋值：组合生成octal_cmd_byte_count及其相邻接口信号，表达握手、选择或地址关系。
        assign octal_cmd_byte_count = 24'd0;
        assign octal_rx_ready       = 1'b0;
        weight_cache_ab #(
            .CACHE_BLOCKS                (WEIGHT_CACHE_BLOCKS),
            .CACHE_INDEX_WIDTH           (WEIGHT_CACHE_INDEX_WIDTH),
            .FIRST_LAYER_FLASH_BASE      (FIRST_LAYER_FLASH_BASE),
            .FIRST_LAYER_BLOCKS          (16'd4),
            .SPI_DIVIDER                 (FLASH_SPI_DIVIDER)
        ) u_weight_cache (
            .clk                         (clk),
            .rst_n                       (rst_n),
            .req_valid                   (wgt_rd_req_valid),
            .req_ready                   (wgt_rd_req_ready),
            .req_addr                    (wgt_rd_addr),
            .rsp_valid                   (wgt_rd_rsp_valid),
            .rsp_ready                   (wgt_rd_rsp_ready),
            .rsp_data                    (wgt_rd_data),
            .prefetch_valid              (weight_prefetch_valid),
            .prefetch_ready              (weight_prefetch_ready),
            .prefetch_flash_base         (weight_prefetch_flash_base),
            .prefetch_block_count        (weight_prefetch_block_count),
            .prefetch_target_bank        (weight_prefetch_target_bank),
            .prefetch_done               (weight_prefetch_done),
            .activate_valid              (weight_activate_valid),
            .activate_ready              (weight_activate_ready),
            .activate_first_layer        (weight_activate_first_layer),
            .activate_bank               (weight_activate_bank),
            .active_first_layer          (active_weight_is_first_layer),
            .active_bank                 (active_weight_bank),
            .active_bank_valid           (active_weight_bank_valid),
            .first_layer_ready           (first_layer_weight_ready),
            .cache_busy                  (flash_busy),
            .cache_error                 (flash_error),
            .flash_csn                   (flash_csn),
            .flash_sclk                  (flash_sclk),
            .flash_mosi                  (flash_mosi),
            .flash_miso                  (flash_miso)
        );

        // 连续赋值：组合生成flash_wp及其相邻接口信号，表达握手、选择或地址关系。
        assign flash_wp   = 1'bz;
        assign flash_hold = 1'bz;
    end
endgenerate
endmodule
