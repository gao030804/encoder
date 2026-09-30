//=============================================================================
// 模块：depthwise_conv1d_controller
// 功能：专用 8 通道 Depthwise Conv1d 数据通路。
//
// 每次从 Activation SRAM 读取同一时刻的 4 个相邻通道；两个 half 完成
// 8 个输出通道的一次 kernel tap。权重仍使用 [OG][KG][4][8] 的 256-bit
// 块布局，但 lane n 只与输入通道 OG*8+n 相乘，绝不跨通道规约。
// 该实现以功能正确和接口复用为优先，之后可替换为 8/16-lane DW MAC。
//=============================================================================

// -------------------------------------------------------------------------
// 详细阅读说明：
// - Depthwise循环为output_group -> m -> k_group -> tap -> half；half两拍读取8个相邻通道。
// - lane n仅执行input[channel=n]*weight[channel=n]，不存在Dense卷积的输入通道求和。
// - padding通过time_valid和4-bit mask实现；无效时间不贡献乘积。
// - 八路结果分别用INT32累加，之后复用逐通道Bias/Requant和64-bit写回格式。
// -------------------------------------------------------------------------
// [中文注释-自动补充]
// 模块作用：Depthwise Conv1d控制器。
// 关键变量/接口：每输出通道只与同通道卷积；当前循环按output_group→m→k_group执行并直接进行INT32累加。
// 握手约定：valid与ready在同一上升沿同时为1才完成一次传输；反压期间数据必须保持。
// 位宽约定：地址通常按Byte计，Weight块为256 bit，Activation/Weight基本元素为signed INT8。
// -----------------------------------------------------------------------------
module depthwise_conv1d_controller #(
    parameter ADDR_WIDTH = 32
) (
    input                                       clk,
    input                                       rst_n,
    input                                       start,
    output reg                                  busy,
    output reg                                  done,
    output reg                                  error,
    input              [31:0]                   cfg_input_base,
    input              [31:0]                   cfg_output_base,
    input              [31:0]                   cfg_weight_base,
    input              [8:0]                    cfg_cin,
    input              [8:0]                    cfg_cout,
    input              [4:0]                    cfg_kernel,
    input              [3:0]                    cfg_stride,
    input              [3:0]                    cfg_dilation,
    input              [7:0]                    cfg_left_pad,
    input              [8:0]                    cfg_input_length,
    input              [8:0]                    cfg_output_length,
    input              [9:0]                    cfg_k_groups,
    input              [5:0]                    cfg_n_groups,
    input                                       cfg_relu_enable,
    output                                      act_rd_req_valid,
    input                                       act_rd_req_ready,
    output             [ADDR_WIDTH-1:0]         act_rd_addr0, act_rd_addr1, act_rd_addr2, act_rd_addr3,
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

localparam S_IDLE=0,S_CHECK=1,S_PARAM_REQ=2,S_PARAM_WAIT=3,S_M_INIT=4,
S_W_REQ=5,S_W_WAIT=6,S_ACT_REQ=7,S_ACT_WAIT=8,
S_POST_SEND=9,S_POST_WAIT=10,S_WRITE_WAIT=11,S_DONE=12,S_ERROR=13;
reg [3:0] state;
reg [5:0] og;
reg [8:0] m;
reg [9:0] kg;
reg [1:0] tap;
reg half;
reg [255:0] weight_q,mult_q;
reg [511:0] bias_q;
reg [63:0] shift_q;
reg [63:0] zp_q;
reg signed [39:0] acc0,acc1,acc2,acc3,acc4,acc5,acc6,acc7;

// -------------------------------------------------------------------------
// Depthwise地址生成
// -------------------------------------------------------------------------
// 当前核位置：kernel_index = 4*kg + tap。
// 对应输入时间：time = m*stride + kernel_index*dilation - left_pad。
// activation采用time-major布局，因此字节地址为：
// input_base + time*Cin + channel。
wire signed [19:0] time_index =
    $signed({1'b0,m})*$signed({1'b0,cfg_stride}) +
    $signed({1'b0,kg,2'b00}+tap)*$signed({1'b0,cfg_dilation}) -
$signed({1'b0,cfg_left_pad});
wire time_valid = (time_index >= 0) && (time_index < $signed({1'b0,cfg_input_length}));
wire [9:0] channel_base = ({4'd0,og}<<3) + (half ? 10'd4 : 10'd0);
wire [31:0] activation_offset = time_index*cfg_cin + channel_base;
// 连续赋值：组合生成act_rd_addr0及其相邻接口信号，表达握手、选择或地址关系。
assign act_rd_addr0 = cfg_input_base + (activation_offset << 1);
assign act_rd_addr1 = act_rd_addr0 + 2'd2;
// 连续赋值：组合生成act_rd_addr2及其相邻接口信号，表达握手、选择或地址关系。
assign act_rd_addr2 = act_rd_addr0 + 3'd4;
assign act_rd_addr3 = act_rd_addr0 + 3'd6;
// 连续赋值：组合生成act_rd_mask及其相邻接口信号，表达握手、选择或地址关系。
assign act_rd_mask[0] = time_valid && (channel_base     < cfg_cout);
assign act_rd_mask[1] = time_valid && (channel_base+1  < cfg_cout);
// 连续赋值：组合生成act_rd_mask及其相邻接口信号，表达握手、选择或地址关系。
assign act_rd_mask[2] = time_valid && (channel_base+2  < cfg_cout);
assign act_rd_mask[3] = time_valid && (channel_base+3  < cfg_cout);
// 连续赋值：组合生成act_rd_req_valid及其相邻接口信号，表达握手、选择或地址关系。
assign act_rd_req_valid = (state==S_ACT_REQ);
assign act_rd_rsp_ready = (state==S_ACT_WAIT);

// 每个256-bit权重块保存4个kernel tap、8个输出通道：
// weight[8*(tap*8+output_lane) +: 8]。
wire [31:0] block_number = og*cfg_k_groups + kg;
// 连续赋值：组合生成wgt_rd_addr及其相邻接口信号，表达握手、选择或地址关系。
assign wgt_rd_addr = cfg_weight_base + (block_number<<5);
assign wgt_rd_req_valid = (state==S_W_REQ);
// 连续赋值：组合生成wgt_rd_rsp_ready及其相邻接口信号，表达握手、选择或地址关系。
assign wgt_rd_rsp_ready = (state==S_W_WAIT);
assign param_rd_req_valid = (state==S_PARAM_REQ);
// 连续赋值：组合生成param_rd_rsp_ready及其相邻接口信号，表达握手、选择或地址关系。
assign param_rd_rsp_ready = (state==S_PARAM_WAIT);
assign param_rd_output_group = og;
wire [319:0] acc_bus={acc7,acc6,acc5,acc4,acc3,acc2,acc1,acc0};
wire post_in_ready,post_valid,post_ready;
wire [127:0] post_data;

// 8路INT32累加值统一进入逐输出通道后处理：
// accumulator + bias -> multiplier/shift -> saturation -> INT8。
postprocess_8lane u_post (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .in_valid                    (state==S_POST_SEND),
    .in_ready                    (post_in_ready),
    .acc_data                    (acc_bus),
    .bias_data                   (bias_q),
    .mult_data                   (mult_q),
    .shift_data                  (shift_q),
    .zero_point_data             (zp_q),
    .post_valid                  (post_valid),
    .post_ready                  (post_ready),
    .post_data                   (post_data)
);

integer n;
reg [127:0] activated_data;
reg [7:0] write_strobe;

// 组合逻辑：根据当前输入计算activated_data、write_strobe、n、post_ready、write_offset；本逻辑块不保存跨周期状态。
always @(*) begin
    activated_data=post_data;
    write_strobe=0;
    for(n=0;n<8;n=n+1) begin
        if (((og<<3)+n)<cfg_cout) write_strobe[n]=1'b1;
        if (cfg_relu_enable && post_data[16*n+15]) activated_data[16*n +: 16]=16'd0;
    end
end

wire writer_in_ready;
wire writer_vector_done;
// 连续赋值：组合生成post_ready及其相邻接口信号，表达握手、选择或地址关系。
assign post_ready=writer_in_ready;
wire [31:0] write_offset=m*cfg_cout+({26'd0,og}<<3);

// 输出同样采用time-major布局：output_base + m*Cout + output_group*8。
// write_strobe负责屏蔽最后一个不足8通道的output group。
output_writer_8lane #(.ADDR_WIDTH(ADDR_WIDTH)) u_writer (
    .clk                         (clk),
    .rst_n                       (rst_n),
    .in_valid                    (post_valid),
    .in_ready                    (writer_in_ready),
    .in_addr                     (cfg_output_base+(write_offset << 1)),
    .in_data                     (activated_data),
    .in_strb                     (write_strobe),
    .out_wr_valid                (out_wr_valid),
    .out_wr_ready                (out_wr_ready),
    .out_wr_addr                 (out_wr_addr),
    .out_wr_data                 (out_wr_data),
    .out_wr_strb                 (out_wr_strb),
    .vector_done                 (writer_vector_done)
);

// -------------------------------------------------------------------------
// 主状态机
// -------------------------------------------------------------------------
// 循环次序：output_group -> output_time(m) -> kernel_group -> tap -> half。
// half=0读取当前8通道的低4通道；half=1读取高4通道。
// 每个请求状态与等待响应状态分开，确保SRAM任意延迟和反压下不丢数据。
// 时序逻辑：在时钟沿更新state、busy、done、error、og、m；复位分支负责恢复确定的空闲状态。
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state<=S_IDLE;
        busy<=0;
        done<=0;
        error<=0;
        og<=0;
        m<=0;
        kg<=0;
        tap<=0;
        half<=0;
        weight_q<=0;
        bias_q<=0;
        mult_q<=0;
        shift_q<=0;
        zp_q<=0;
        acc0<=0;
        acc1<=0;
        acc2<=0;
        acc3<=0;
        acc4<=0;
        acc5<=0;
        acc6<=0;
        acc7<=0;
    end else begin
        done<=0;
        error<=0;
        case(state)
            S_IDLE: if(start) begin
                busy<=1;
                og<=0;
                state<=S_CHECK;
            end
            S_CHECK: begin
                if((cfg_cin==cfg_cout)&&(cfg_kernel!=0)&&(cfg_stride!=0)&&
                    (cfg_dilation!=0)&&(cfg_output_length!=0)&&
                    (cfg_k_groups==((cfg_kernel+3)>>2))&&
                    (cfg_n_groups==((cfg_cout+7)>>3))) state<=S_PARAM_REQ;
                else state<=S_ERROR;
            end
            S_PARAM_REQ: if(param_rd_req_ready) state<=S_PARAM_WAIT;
            S_PARAM_WAIT: if(param_rd_rsp_valid) begin
                bias_q<=param_bias_data;
                mult_q<=param_mult_data;
                shift_q<=param_shift_data;
                zp_q<=param_zero_point_data;
                m<=0;
                state<=S_M_INIT;
            end
            S_M_INIT: begin
                kg<=0;
                tap<=0;
                half<=0;
                acc0<=0;
                acc1<=0;
                acc2<=0;
                acc3<=0;
                acc4<=0;
                acc5<=0;
                acc6<=0;
                acc7<=0;
                state<=S_W_REQ;
            end
            S_W_REQ: if(wgt_rd_req_ready) state<=S_W_WAIT;
            S_W_WAIT: if(wgt_rd_rsp_valid) begin
                weight_q<=wgt_rd_data;
                tap<=0;
                half<=0;
                state<=S_ACT_REQ;
            end
            S_ACT_REQ: if(act_rd_req_ready) state<=S_ACT_WAIT;
            S_ACT_WAIT: if(act_rd_rsp_valid) begin
                if(!half) begin
                    if(act_rd_mask[0]) acc0<=acc0+$signed(act_rd_data[15:0])*$signed(weight_q[8*(tap*8+0) +: 8]);
                    if(act_rd_mask[1]) acc1<=acc1+$signed(act_rd_data[31:16])*$signed(weight_q[8*(tap*8+1) +: 8]);
                    if(act_rd_mask[2]) acc2<=acc2+$signed(act_rd_data[47:32])*$signed(weight_q[8*(tap*8+2) +: 8]);
                    if(act_rd_mask[3]) acc3<=acc3+$signed(act_rd_data[63:48])*$signed(weight_q[8*(tap*8+3) +: 8]);
                    half<=1;
                    state<=S_ACT_REQ;
                end else begin
                    if(act_rd_mask[0]) acc4<=acc4+$signed(act_rd_data[15:0])*$signed(weight_q[8*(tap*8+4) +: 8]);
                    if(act_rd_mask[1]) acc5<=acc5+$signed(act_rd_data[31:16])*$signed(weight_q[8*(tap*8+5) +: 8]);
                    if(act_rd_mask[2]) acc6<=acc6+$signed(act_rd_data[47:32])*$signed(weight_q[8*(tap*8+6) +: 8]);
                    if(act_rd_mask[3]) acc7<=acc7+$signed(act_rd_data[63:48])*$signed(weight_q[8*(tap*8+7) +: 8]);
                    half<=0;
                    if((tap==3)||(({kg,2'b00}+tap+1)>=cfg_kernel)) begin
                        if(kg+1>=cfg_k_groups) state<=S_POST_SEND;
                        else begin
                            kg<=kg+1'b1;
                            state<=S_W_REQ;
                        end
                    end else begin
                        tap<=tap+1'b1;
                        state<=S_ACT_REQ;
                    end
                end
            end
            S_POST_SEND: if(post_in_ready) state<=S_POST_WAIT;
            S_POST_WAIT: if(post_valid&&post_ready) state<=S_WRITE_WAIT;
            // 等待整个8×INT16向量写完，而不是只等待低64-bit写拍。
            S_WRITE_WAIT: if(writer_vector_done) begin
                if(m+1<cfg_output_length) begin
                    m<=m+1'b1;
                    state<=S_M_INIT;
                end
                else if(og+1<cfg_n_groups) begin
                    og<=og+1'b1;
                    state<=S_PARAM_REQ;
                end
                else state<=S_DONE;
            end
            S_DONE: begin
                busy<=0;
                done<=1;
                state<=S_IDLE;
            end
            S_ERROR: begin
                busy<=0;
                error<=1;
                state<=S_IDLE;
            end
            default: state<=S_ERROR;
        endcase
    end
end

endmodule
