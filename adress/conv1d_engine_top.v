// ============================================================================
// 中文阅读导引（当前实现）
// 单层Conv1d接口封装。配置由controller在空闲时接收start后锁存。
// Activation读接口每次返回4个INT8；Weight读接口每次返回32个INT8；输出每次写8个INT8。
// 读请求和读响应有独立握手，发出请求不等于SRAM数据已经返回。
// ============================================================================

//=============================================================================
// Conv1d 计算顶层：按 cfg_is_depthwise 在 Dense 4x8 脉动阵列与专用
// Depthwise 8-lane MAC 之间选择。两条路径复用同一组 SRAM valid/ready 接口。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - start时锁存cfg_is_depthwise：0选择4x8 Dense Mesh，1选择专用Depthwise MAC。
// - Activation、Weight、Parameter和Output接口均进行双向valid/ready选择，未选路径不能消费响应。
// - Dense路径覆盖普通卷积及所有低秩1x1卷积；Depthwise路径禁止跨输入通道规约。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：Dense/Depthwise卷积引擎选择层。
// 关键变量/接口：cfg_is_depthwise选择控制器；未选路径的ready/valid被隔离，外部接口保持统一。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module conv1d_engine_top #(
    parameter ADDR_WIDTH=32, parameter M_TAG_WIDTH=9, parameter MAX_M=320
) (
    input                                       clk,
    input                                       rst_n,
    input                                       start,
    output                                      busy,
    output                                      done,
    output                                      error,
    input              [31:0]                   cfg_input_base,cfg_output_base,cfg_weight_base,
    input              [8:0]                    cfg_cin,cfg_cout,
    input              [4:0]                    cfg_kernel,
    input              [3:0]                    cfg_stride,cfg_dilation,
    input              [7:0]                    cfg_left_pad,
    input              [8:0]                    cfg_input_length,cfg_output_length,
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
    output             [ADDR_WIDTH-1:0]         act_rd_addr0,act_rd_addr1,act_rd_addr2,act_rd_addr3,
    output             [3:0]                    act_rd_mask,
    input                                       act_rd_rsp_valid,
    output                                      act_rd_rsp_ready,
    input              [63:0]                   act_rd_data,
    output                                      wgt_rd_req_valid,
    input                                       wgt_rd_req_ready,
    output             [ADDR_WIDTH-1:0]         wgt_rd_addr,
    input                                       wgt_rd_rsp_valid,
    output                                      wgt_rd_rsp_ready,
    input              [255:0]                  wgt_rd_data,
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
    output             [7:0]                    out_wr_strb
);

// start 时锁存路径选择；运行中 cfg_is_depthwise 的变化不会切换数据通路。
reg select_dw;

// 只在引擎空闲并接受start时锁存模式。这样即使外部控制器提前更新下一层
// cfg_is_depthwise，也不会让正在执行的层从Dense通路跳到Depthwise通路。
// 时序逻辑：在时钟沿更新select_dw；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n)
if(!rst_n) select_dw<=1'b0;
else if(start&&!busy) select_dw<=cfg_is_depthwise;
wire dense_start=start&&!cfg_is_depthwise;
wire dw_start=start&&cfg_is_depthwise;
wire dbusy,ddone,derror,dav,darr,dwv,dense_w_rsp_ready,dpv,dprspready,dov;
wire [ADDR_WIDTH-1:0] da0,da1,da2,da3,dwa,doa;
wire [3:0] dam;
wire [5:0] dpog;
wire [63:0] dod;
wire [7:0] dos;
wire dwbusy,dwdone,dwerror,dw_actv,dw_wgtv,dwarr,dwrspready,dwpv,dwprspready,dwov;
wire [ADDR_WIDTH-1:0] dwa0,dwa1,dwa2,dwa3,dwwa,dwoa;
wire [3:0] dwam;
wire [5:0] dwpog;
wire [63:0] dwod;
wire [7:0] dwos;

// -------------------------------------------------------------------------
// 外部SRAM接口复用
// -------------------------------------------------------------------------
// select_dw=0：请求和响应连接4×8脉动阵列控制器。
// select_dw=1：请求和响应连接专用Depthwise控制器。
// ready/valid也必须同时选择，不能只复用data，否则未选中的控制器会误计数。
// 连续赋值：组合生成busy及其相邻接口信号，表达握手、选择或地址关系。
assign busy=select_dw?dwbusy:dbusy;
assign done=select_dw?dwdone:ddone;
// 连续赋值：组合生成error及其相邻接口信号，表达握手、选择或地址关系。
assign error=select_dw?dwerror:derror;
assign act_rd_req_valid=select_dw?dw_actv:dav;
// 连续赋值：组合生成act_rd_addr0及其相邻接口信号，表达握手、选择或地址关系。
assign act_rd_addr0=select_dw?dwa0:da0;
assign act_rd_addr1=select_dw?dwa1:da1;
// 连续赋值：组合生成act_rd_addr2及其相邻接口信号，表达握手、选择或地址关系。
assign act_rd_addr2=select_dw?dwa2:da2;
assign act_rd_addr3=select_dw?dwa3:da3;
// 连续赋值：组合生成act_rd_mask及其相邻接口信号，表达握手、选择或地址关系。
assign act_rd_mask=select_dw?dwam:dam;
assign act_rd_rsp_ready=select_dw?dwarr:darr;
// 连续赋值：组合生成wgt_rd_req_valid及其相邻接口信号，表达握手、选择或地址关系。
assign wgt_rd_req_valid=select_dw?dw_wgtv:dwv;
assign wgt_rd_addr=select_dw?dwwa:dwa;
// 连续赋值：组合生成wgt_rd_rsp_ready及其相邻接口信号，表达握手、选择或地址关系。
assign wgt_rd_rsp_ready=select_dw?dwrspready:dense_w_rsp_ready;
assign param_rd_req_valid=select_dw?dwpv:dpv;
// 连续赋值：组合生成param_rd_output_group及其相邻接口信号，表达握手、选择或地址关系。
assign param_rd_output_group=select_dw?dwpog:dpog;
assign param_rd_rsp_ready=select_dw?dwprspready:dprspready;
// 连续赋值：组合生成out_wr_valid及其相邻接口信号，表达握手、选择或地址关系。
assign out_wr_valid=select_dw?dwov:dov;
assign out_wr_addr=select_dw?dwoa:doa;
// 连续赋值：组合生成out_wr_data及其相邻接口信号，表达握手、选择或地址关系。
assign out_wr_data=select_dw?dwod:dod;
assign out_wr_strb=select_dw?dwos:dos;

// Dense、普通Pointwise及低秩Pointwise均使用该实例。
conv1d_controller #(.ADDR_WIDTH(ADDR_WIDTH),
    .M_TAG_WIDTH                 (M_TAG_WIDTH),
    .MAX_M                       (MAX_M)) u_dense (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .start                       (dense_start),
    .busy                        (dbusy),
    .done                        (ddone),
    .error                       (derror),
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
    .cfg_elu_enable              (cfg_elu_enable),
    .cfg_relu_enable             (cfg_relu_enable),
    .elu_lut_cfg_valid           (elu_lut_cfg_valid),
    .elu_lut_cfg_ready           (elu_lut_cfg_ready),
    .elu_lut_cfg_addr            (elu_lut_cfg_addr),
    .elu_lut_cfg_wdata           (elu_lut_cfg_wdata),
    .act_rd_req_valid            (dav),
    .act_rd_req_ready            (act_rd_req_ready&&!select_dw),
    .act_rd_addr0                (da0),
    .act_rd_addr1                (da1),
    .act_rd_addr2                (da2),
    .act_rd_addr3                (da3),
    .act_rd_mask                 (dam),
    .act_rd_rsp_valid            (act_rd_rsp_valid&&!select_dw),
    .act_rd_rsp_ready            (darr),
    .act_rd_data                 (act_rd_data),
    .wgt_rd_req_valid            (dwv),
    .wgt_rd_req_ready            (wgt_rd_req_ready&&!select_dw),
    .wgt_rd_addr                 (dwa),
    .wgt_rd_rsp_valid            (wgt_rd_rsp_valid&&!select_dw),
    .wgt_rd_rsp_ready            (dense_w_rsp_ready),
    .wgt_rd_data                 (wgt_rd_data),
    .param_rd_req_valid          (dpv),
    .param_rd_req_ready          (param_rd_req_ready&&!select_dw),
    .param_rd_output_group       (dpog),
    .param_rd_rsp_valid          (param_rd_rsp_valid&&!select_dw),
    .param_rd_rsp_ready          (dprspready),
    .param_bias_data             (param_bias_data),
    .param_mult_data             (param_mult_data),
    .param_shift_data            (param_shift_data),
    .param_zero_point_data       (param_zero_point_data),
    .out_wr_valid                (dov),
    .out_wr_ready                (out_wr_ready&&!select_dw),
    .out_wr_addr                 (doa),
    .out_wr_data                 (dod),
    .out_wr_strb                 (dos));

// Depthwise层使用独立MAC通路。其每个输出lane只读取同编号输入通道，
// 因而不会发生普通Dense卷积中的跨通道规约。
depthwise_conv1d_controller #(.ADDR_WIDTH(ADDR_WIDTH)) u_depthwise (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .start                       (dw_start),
    .busy                        (dwbusy),
    .done                        (dwdone),
    .error                       (dwerror),
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
    .cfg_k_groups                (cfg_k_groups),
    .cfg_n_groups                (cfg_n_groups),
    .cfg_relu_enable             (cfg_relu_enable),
    .act_rd_req_valid            (dw_actv),
    .act_rd_req_ready            (act_rd_req_ready&&select_dw),
    .act_rd_addr0                (dwa0),
    .act_rd_addr1                (dwa1),
    .act_rd_addr2                (dwa2),
    .act_rd_addr3                (dwa3),
    .act_rd_mask                 (dwam),
    .act_rd_rsp_valid            (act_rd_rsp_valid&&select_dw),
    .act_rd_rsp_ready            (dwarr),
    .act_rd_data                 (act_rd_data),
    .wgt_rd_req_valid            (dw_wgtv),
    .wgt_rd_req_ready            (wgt_rd_req_ready&&select_dw),
    .wgt_rd_addr                 (dwwa),
    .wgt_rd_rsp_valid            (wgt_rd_rsp_valid&&select_dw),
    .wgt_rd_rsp_ready            (dwrspready),
    .wgt_rd_data                 (wgt_rd_data),
    .param_rd_req_valid          (dwpv),
    .param_rd_req_ready          (param_rd_req_ready&&select_dw),
    .param_rd_output_group       (dwpog),
    .param_rd_rsp_valid          (param_rd_rsp_valid&&select_dw),
    .param_rd_rsp_ready          (dwprspready),
    .param_bias_data             (param_bias_data),
    .param_mult_data             (param_mult_data),
    .param_shift_data            (param_shift_data),
    .param_zero_point_data       (param_zero_point_data),
    .out_wr_valid                (dwov),
    .out_wr_ready                (out_wr_ready&&select_dw),
    .out_wr_addr                 (dwoa),
    .out_wr_data                 (dwod),
    .out_wr_strb                 (dwos));
endmodule
