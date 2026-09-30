`timescale 1ns/1ps
// W8A16投影 + V1整数RVQ。输入来自Encoder真实结果读端口；不使用Golden驱动。
// SRAM字节布局：0000投影W(2048B),0800投影参数(32x16B),
// 0A00码本(40960B),AA00 norm(5120B),BE00逐级参数(9x32B)。
// 每个投影参数记录：bias INT64容器、M32、S8、3Byte保留。
// 每级记录前10Byte：next M32/S8、output M32/S8；其余保留。
module rvq_backend_w8a16 #(
    parameter COMPUTE_VECTOR = 1 // 完整系统默认关闭；软件根据indices和码本重建
) (
    input clk, input rst_n,
    input boot_write, input [15:0] boot_index, input [255:0] boot_data,
    input start, output busy, output reg error, output reg configured,
    input [31:0] latent_base,
    output latent_req_valid, input latent_req_ready,
    output [31:0] latent_addr,
    input latent_rsp_valid, output latent_rsp_ready, input [63:0] latent_data,
    output reg result_valid, input result_ready,
    output reg [63:0] result_indices, output reg [511:0] result_vector,
    // 只读观察口：每个投影输出、候选score、残差更新、级完成均可逐项对拍。
    output reg projection_valid, output reg [4:0] projection_channel,
    output reg signed [15:0] projection_data,
    output reg score_valid, output reg [3:0] trace_stage,
    output reg [7:0] trace_candidate, output reg signed [31:0] trace_score,
    output reg update_valid, output reg [4:0] trace_dimension,
    output reg signed [15:0] trace_residual,
    output reg signed [7:0] trace_codeword,
    output reg signed [16:0] trace_subtract,
    output reg signed [15:0] trace_next,
    output reg stage_valid, output reg [7:0] trace_index
);
localparam IDLE=0,LREQ=1,LWAIT=2,PPREQ=3,PPWAIT=4,PWREQ=5,PWWAIT=6,
           PMAC=7,PQUANT=8,RPREQ=9,RPWAIT=10,NREQ=11,NWAIT=12,
           BREQ=13,BWAIT=14,DOT=15,SCORE=16,SREQ=17,SWAIT=18,
           UPDATE=19,STAGE_END=20,OUTPUT=21,HOLD=22;
reg [4:0] state;
reg [3:0] latent_group, stage;
reg [4:0] channel, dimension;
reg half;
reg [1:0] group8;
reg [7:0] candidate, best_index;
reg signed [31:0] best_score, norm, dot_acc;
reg signed [39:0] projection_acc, bias;
reg signed [31:0] multiplier, next_m, output_m;
reg [7:0] shift, next_s, output_s;
reg signed [15:0] x [0:63];
reg signed [15:0] residual [0:31];
reg signed [31:0] output_sum [0:31];
reg [255:0] word_q;
reg [15:0] loaded_blocks;
integer j;
integer lane;

// 每次只读一个256bit逻辑字，适配器拆成2个128bit宏；3组深度bank。
reg mem_req;
reg [15:0] byte_address;
wire [255:0] mem_data;
wire mem_valid;
wire [10:0] mem_address = boot_write ? boot_index[10:0] : byte_address[15:5];
opentitan_sram_1p_adapter #(.WIDTH(256),.DEPTH(1536)) u_storage (
    .clk_i(clk),.rst_ni(rst_n),.req_i(boot_write || mem_req),
    .write_i(boot_write),.addr_i(mem_address),.wdata_i(boot_data),
    .wmask_i(32'hffffffff),.rdata_o(mem_data),.rvalid_o(mem_valid)
);
// stage0有256项，后续各128项。地址只遍历实际码字，不搜索padding。
wire [10:0] stage_base = (stage==0) ? 11'd0 : 11'd256+(stage-1)*11'd128;
wire [10:0] entry_id = stage_base+candidate;
wire [7:0] last_candidate = (stage==0) ? 8'd255 : 8'd127;
assign busy=(state!=IDLE);
assign latent_req_valid=(state==LREQ);
assign latent_rsp_ready=(state==LWAIT);
assign latent_addr=latent_base+{25'd0,latent_group,3'b000};
always @* begin
    mem_req=1'b1; byte_address=0;
    case(state)
        PPREQ: byte_address=16'h0800+channel*16;
        PWREQ: byte_address=channel*64+half*32;
        RPREQ: byte_address=16'hbe00+stage*32;
        NREQ: byte_address=16'haa00+entry_id*4;
        BREQ: byte_address=16'h0a00+entry_id*32;
        SREQ: byte_address=16'h0a00+(stage_base+best_index)*32;
        default: mem_req=0;
    endcase
end

// 8个乘法器按8维一组复用。投影每输出64维，RVQ每候选32维。
reg signed [39:0] mac_partial;
reg signed [15:0] mac_a;
reg signed [7:0] mac_b;
reg signed [23:0] mac_product;
wire signed [39:0] projection_partial=mac_partial;
wire signed [31:0] dot_partial=mac_partial[31:0];
always @* begin
    mac_partial=0;mac_a=0;mac_b=0;mac_product=0;
    for(lane=0;lane<8;lane=lane+1) begin
        mac_a=(state==PMAC) ? x[half*32+group8*8+lane] : residual[group8*8+lane];
        mac_b=word_q[8*(group8*8+lane)+:8];
        mac_product=mac_a*mac_b;
        mac_partial=mac_partial+mac_product;
    end
end

// 宽乘积后按绝对值舍入，再恢复符号，保证负数half-away语义。
function signed [15:0] sat16;
    input signed [79:0] v;
    begin
        if(v>32767) sat16=16'sh7fff;
        else if(v< -32768) sat16=16'sh8000;
        else sat16=v[15:0];
    end
endfunction
function signed [79:0] round80;
    input signed [79:0] p;
    input [7:0] s;
    reg [79:0] magnitude, rounded;
    begin
        magnitude=p<0 ? -p : p;
        rounded=(s==0) ? magnitude : (magnitude+(80'd1<<(s-1)))>>s;
        round80=p<0 ? -$signed(rounded) : $signed(rounded);
    end
endfunction
function signed [63:0] rq64;
    input signed [16:0] v;
    input signed [31:0] m;
    input [7:0] s;
    reg signed [63:0] p;
    reg [63:0] mag, rounded;
    begin
        p=v*m; mag=p<0 ? -p : p;
        rounded=mag>>s;
        if(s!=0 && (mag & ((64'd1<<s)-1)) >= (64'd1<<(s-1))) rounded=rounded+1;
        rq64=p<0 ? -$signed(rounded) : $signed(rounded);
    end
endfunction
wire signed [40:0] biased=$signed(projection_acc)+$signed(bias);
wire signed [39:0] biased40=(biased>41'sd549755813887) ? 40'sh7fffffffff :
    (biased < -41'sd549755813888) ? 40'sh8000000000 : biased[39:0];
wire signed [79:0] projected_product=$signed(biased40)*$signed(multiplier);
wire signed [31:0] score_value=norm-(dot_acc<<<1);
wire signed [7:0] selected_value=word_q[dimension*8+:8];
wire signed [16:0] sub_value=$signed({residual[dimension][15],residual[dimension]})-
    $signed({{9{selected_value[7]}},selected_value});
wire signed [63:0] next_value=rq64(sub_value,next_m,next_s);
wire signed [63:0] output_value=rq64({{9{selected_value[7]}},selected_value},output_m,output_s);
wire signed [63:0] sum_value=$signed(output_sum[dimension])+output_value;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state<=IDLE;error<=0;configured<=0;loaded_blocks<=0;
        result_valid<=0;result_indices<=0;result_vector<=0;
        projection_valid<=0;score_valid<=0;update_valid<=0;stage_valid<=0;
        latent_group<=0;stage<=0;channel<=0;dimension<=0;half<=0;group8<=0;
        candidate<=0;best_index<=0;best_score<=0;norm<=0;dot_acc<=0;
        projection_acc<=0;bias<=0;multiplier<=0;shift<=0;
        next_m<=0;next_s<=0;output_m<=0;output_s<=0;word_q<=0;
        trace_stage<=0;trace_candidate<=0;trace_score<=0;trace_index<=0;
        trace_dimension<=0;trace_residual<=0;trace_codeword<=0;trace_subtract<=0;trace_next<=0;
        projection_channel<=0;projection_data<=0;
    end else begin
        projection_valid<=0;score_valid<=0;update_valid<=0;stage_valid<=0;
        if(boot_write) begin
            if(busy || boot_index!=loaded_blocks || boot_index>=1536) error<=1;
            loaded_blocks<=loaded_blocks+1'b1;
            if(boot_index==1535) configured<=1;
        end
        case(state)
        IDLE: if(start) begin
            if(!configured) error<=1;
            else begin
                latent_group<=0;result_indices<=0;state<=LREQ;
                if(COMPUTE_VECTOR) for(j=0;j<32;j=j+1) output_sum[j]<=0;
            end
        end
        LREQ: if(latent_req_ready) state<=LWAIT;
        LWAIT: if(latent_rsp_valid) begin
            for(j=0;j<4;j=j+1) x[latent_group*4+j]<=latent_data[j*16+:16];
            if(latent_group==15) begin channel<=0;state<=PPREQ;end
            else begin latent_group<=latent_group+1'b1;state<=LREQ;end
        end
        PPREQ: state<=PPWAIT;
        PPWAIT: if(mem_valid) begin
            bias<=mem_data[channel[0]*128+:40];
            multiplier<=mem_data[channel[0]*128+64+:32];
            shift<=mem_data[channel[0]*128+96+:8];
            if(mem_data[channel[0]*128+96+:8]>63) error<=1;
            projection_acc<=0;half<=0;state<=PWREQ;
        end
        PWREQ: state<=PWWAIT;
        PWWAIT: if(mem_valid) begin word_q<=mem_data;group8<=0;state<=PMAC;end
        PMAC: begin
            projection_acc<=projection_acc+projection_partial;
            if(group8==3) begin
                if(half) state<=PQUANT;
                else begin half<=1;state<=PWREQ;end
            end else group8<=group8+1'b1;
        end
        PQUANT: begin
            residual[channel]<=sat16(round80(projected_product,shift));
            projection_data<=sat16(round80(projected_product,shift));
            projection_channel<=channel;projection_valid<=1;
            if(channel==31) begin stage<=0;state<=RPREQ;end
            else begin channel<=channel+1'b1;state<=PPREQ;end
        end
        RPREQ: state<=RPWAIT;
        RPWAIT: if(mem_valid) begin
            next_m<=mem_data[31:0];next_s<=mem_data[39:32];
            output_m<=mem_data[71:40];output_s<=mem_data[79:72];
            if(mem_data[39:32]>63 || mem_data[79:72]>63) error<=1;
            candidate<=0;best_score<=32'sh7fffffff;best_index<=0;state<=NREQ;
        end
        NREQ: state<=NWAIT;
        NWAIT: if(mem_valid) begin norm<=mem_data[entry_id[2:0]*32+:32];state<=BREQ;end
        BREQ: state<=BWAIT;
        BWAIT: if(mem_valid) begin word_q<=mem_data;group8<=0;dot_acc<=0;state<=DOT;end
        DOT: begin
            dot_acc<=dot_acc+dot_partial;
            if(group8==3) state<=SCORE;else group8<=group8+1'b1;
        end
        SCORE: begin
            trace_stage<=stage;trace_candidate<=candidate;trace_score<=score_value;score_valid<=1;
            // 从小到大遍历，严格小于才替换，因此平分时保留最小index。
            if(score_value<best_score) begin best_score<=score_value;best_index<=candidate;end
            if(candidate==last_candidate) state<=SREQ;
            else begin candidate<=candidate+1'b1;state<=NREQ;end
        end
        SREQ: state<=SWAIT;
        SWAIT: if(mem_valid) begin word_q<=mem_data;dimension<=0;state<=UPDATE;end
        UPDATE: begin
            trace_stage<=stage;trace_dimension<=dimension;trace_residual<=residual[dimension];
            trace_codeword<=selected_value;trace_subtract<=sub_value;
            trace_next<=sat16(next_value);update_valid<=1;
            if(stage!=8) residual[dimension]<=sat16(next_value);
            // 只在最后统一饱和；禁止每级先sat16再相加。
            if(COMPUTE_VECTOR) begin
                output_sum[dimension]<=sum_value[31:0];
                if(sum_value>2147483647 || sum_value< -64'sd2147483648) error<=1;
            end
            if(dimension==31) state<=STAGE_END;else dimension<=dimension+1'b1;
        end
        STAGE_END: begin
            stage_valid<=1;trace_stage<=stage;trace_index<=best_index;
            if(stage==0) result_indices[7:0]<=best_index;
            else result_indices[8+(stage-1)*7+:7]<=best_index[6:0];
            if(stage==8) state<=OUTPUT;
            else begin stage<=stage+1'b1;state<=RPREQ;end
        end
        OUTPUT: begin
            if(COMPUTE_VECTOR)
                for(j=0;j<32;j=j+1) result_vector[j*16+:16]<=sat16(output_sum[j]);
            else result_vector<=0;
            result_valid<=1;state<=HOLD;
        end
        HOLD: if(result_ready) begin result_valid<=0;state<=IDLE;end
        default: begin error<=1;state<=IDLE;end
        endcase
    end
end
endmodule
