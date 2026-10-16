`timescale 1ns/1ps
// 投影分拍专项：真实SRAM接口装载合成参数，不force DUT内部状态。
// 覆盖INT40加Bias饱和、正负乘数、shift=0/1/63、正负半值舍入和INT16饱和。
module tb_qor_p0_projection;
    reg clk=0, rst_n=0, boot_write=0, start=0, result_ready=0;
    reg [15:0] boot_index=0;
    reg [255:0] boot_data=0;
    wire busy,error,configured,latent_req_valid,latent_rsp_ready,result_valid;
    wire [31:0] latent_addr;
    reg latent_rsp_valid=0;
    reg [63:0] latent_data=0;
    wire [63:0] result_indices;
    wire [511:0] result_vector;
    wire projection_valid,score_valid,update_valid,stage_valid;
    wire [4:0] projection_channel,trace_dimension;
    wire signed [15:0] projection_data,trace_residual,trace_next;
    wire [3:0] trace_stage;
    wire [7:0] trace_candidate,trace_index;
    wire signed [31:0] trace_score;
    wire signed [7:0] trace_codeword;
    wire signed [16:0] trace_subtract;
    integer errors=0, projections=0, scores=0, updates=0, stages=0;
    integer cycles=0, block_id,c,n,begin_cycle;
    integer bias_states=0,mul_states=0,quant_states=0;
    reg [31:0] projection_seen=0;
    reg signed [15:0] expected [0:31];
    reg [63:0] held_indices;
    always #10 clk=~clk;

    rvq_backend_w8a16 #(.COMPUTE_VECTOR(1)) dut (
        .clk(clk),.rst_n(rst_n),.boot_write(boot_write),.boot_index(boot_index),
        .boot_data(boot_data),.start(start),.busy(busy),.error(error),.configured(configured),
        .latent_base(32'h1000),.latent_req_valid(latent_req_valid),.latent_req_ready(1'b1),
        .latent_addr(latent_addr),.latent_rsp_valid(latent_rsp_valid),
        .latent_rsp_ready(latent_rsp_ready),.latent_data(latent_data),
        .result_valid(result_valid),.result_ready(result_ready),
        .result_indices(result_indices),.result_vector(result_vector),
        .projection_valid(projection_valid),.projection_channel(projection_channel),
        .projection_data(projection_data),.score_valid(score_valid),.trace_stage(trace_stage),
        .trace_candidate(trace_candidate),.trace_score(trace_score),.update_valid(update_valid),
        .trace_dimension(trace_dimension),.trace_residual(trace_residual),
        .trace_codeword(trace_codeword),.trace_subtract(trace_subtract),.trace_next(trace_next),
        .stage_valid(stage_valid),.trace_index(trace_index)
    );

    function signed [15:0] input_value;
        input integer i;
        begin input_value=(i%2==0) ? 16'sd32767 : -16'sd32768; end
    endfunction
    function signed [7:0] weight_value;
        input integer channel_id,dimension_id;
        begin
            // 常量1权重使64维累加恰为-32；通道0类取-1构造正向溢出。
            weight_value=(channel_id%8==0) ? -1 : 1;
        end
    endfunction
    function signed [39:0] bias_value;
        input integer i;
        begin
            case(i%8)
                0:bias_value=40'sh7fffffffff;
                1:bias_value=40'sh8000000000;
                2:bias_value=40'sd33;  // Acc+Bias=+1
                3:bias_value=40'sd31;  // Acc+Bias=-1
                4:bias_value=40'sd100032;
                5:bias_value=-40'sd100000;
                default:bias_value=40'sd32;
            endcase
        end
    endfunction
    function signed [31:0] multiplier_value;
        input integer i;
        begin
            case(i%8)
                0:multiplier_value=-32'sd2147483647; // bias和负累加构造宽乘积
                1:multiplier_value=32'sh80000000;
                6:multiplier_value=-32'sd3;
                7:multiplier_value=0;
                default:multiplier_value=1;
            endcase
        end
    endfunction
    function [7:0] shift_value;
        input integer i;
        begin
            case(i%8)
                0,1:shift_value=(i<16) ? 0 : 63;
                2,3:shift_value=1;
                default:shift_value=0;
            endcase
        end
    endfunction
    // 独立参考：用除法/余数判断半值，不调用DUT round80实现。
    function signed [15:0] golden_projection;
        input integer i;
        integer d;
        reg signed [40:0] acc,b;
        reg signed [39:0] clamped;
        reg signed [79:0] product,answer;
        reg [79:0] magnitude,divisor,quotient,remainder;
        begin
            acc=0;
            for(d=0;d<64;d=d+1) acc=acc+input_value(d)*weight_value(i,d);
            b=acc+bias_value(i);
            if(b>41'sd549755813887) clamped=40'sh7fffffffff;
            else if(b< -41'sd549755813888) clamped=40'sh8000000000;
            else clamped=b[39:0];
            product=$signed(clamped)*$signed(multiplier_value(i));
            magnitude=(product<0) ? -product : product;
            divisor=80'd1<<shift_value(i);
            quotient=magnitude/divisor;
            remainder=magnitude%divisor;
            if(shift_value(i)!=0 && remainder>=divisor/2) quotient=quotient+1;
            answer=(product<0) ? -$signed(quotient) : $signed(quotient);
            if(answer>32767) golden_projection=32767;
            else if(answer< -32768) golden_projection=-32768;
            else golden_projection=answer[15:0];
        end
    endfunction
    function [255:0] image_block;
        input integer id;
        integer lane_id,ch;
        reg [255:0] v;
        begin
            v=0;
            if(id<64) begin
                for(lane_id=0;lane_id<32;lane_id=lane_id+1)
                    v[8*lane_id+:8]=weight_value(id/2,(id%2)*32+lane_id);
            end else if(id>=64 && id<80) begin
                for(lane_id=0;lane_id<2;lane_id=lane_id+1) begin
                    ch=(id-64)*2+lane_id;
                    v[lane_id*128+:64]=$signed(bias_value(ch));
                    v[lane_id*128+64+:32]=multiplier_value(ch);
                    v[lane_id*128+96+:8]=shift_value(ch);
                end
            end else if(id>=1520 && id<1529) begin
                v[31:0]=1; v[71:40]=1; // 残差/重建scale均为1，码本和norm全零
            end
            image_block=v;
        end
    endfunction
    always @(posedge clk) begin
        cycles=cycles+1;
        if(!rst_n) latent_rsp_valid<=0;
        else begin
            if(latent_rsp_valid && latent_rsp_ready) latent_rsp_valid<=0;
            if(latent_req_valid) begin
                for(n=0;n<4;n=n+1) latent_data[n*16+:16]<=input_value((latent_addr-32'h1000)/2+n);
                latent_rsp_valid<=1;
            end
            if(dut.state==23) bias_states=bias_states+1;
            if(dut.state==24) mul_states=mul_states+1;
            if(dut.state==8) quant_states=quant_states+1;
            if(error) begin $display("[ERROR] RVQ error asserted");errors=errors+1;end
            if(projection_valid) begin
                if(projection_seen[projection_channel] || projection_data!==expected[projection_channel]) begin
                    $display("[ERROR] projection c=%0d expected=%0d actual=%0d",projection_channel,expected[projection_channel],projection_data);
                    errors=errors+1;
                end
                projection_seen[projection_channel]=1;
                projections=projections+1;
            end
            if(score_valid) begin
                if(trace_score!==0) begin $display("[ERROR] nonzero score");errors=errors+1;end
                scores=scores+1;
            end
            if(update_valid) begin
                if(trace_residual!==expected[trace_dimension] || trace_codeword!==0 ||
                   trace_subtract!==$signed(expected[trace_dimension]) || trace_next!==expected[trace_dimension]) begin
                    $display("[ERROR] residual update mismatch stage=%0d dim=%0d",trace_stage,trace_dimension);
                    errors=errors+1;
                end
                updates=updates+1;
            end
            if(stage_valid) begin
                if(trace_index!==0) begin $display("[ERROR] tie must choose index0");errors=errors+1;end
                stages=stages+1;
            end
        end
    end
    initial begin
        for(c=0;c<32;c=c+1) expected[c]=golden_projection(c);
        repeat(4) @(negedge clk);
        rst_n=1;
        for(block_id=0;block_id<1536;block_id=block_id+1) begin
            @(negedge clk);boot_write=1;boot_index=block_id;boot_data=image_block(block_id);
        end
        @(negedge clk);boot_write=0;
        if(!configured) begin $display("[TB_FAIL] boot incomplete");$finish;end
        start=1;begin_cycle=cycles;
        @(negedge clk);start=0;
        wait(result_valid);
        @(negedge clk);held_indices=result_indices;
        repeat(13) begin
            @(negedge clk);
            if(!result_valid || result_indices!==held_indices) errors=errors+1;
        end
        if(result_indices!==0 || result_vector!==0 || projections!=32 || scores!=1280 ||
           updates!=288 || stages!=9 || bias_states!=32 || mul_states!=32 || quant_states!=32) begin
            $display("[ERROR] coverage projections=%0d scores=%0d updates=%0d stages=%0d bias/mul/quant=%0d/%0d/%0d",projections,scores,updates,stages,bias_states,mul_states,quant_states);
            errors=errors+1;
        end
        result_ready=1;
        @(negedge clk);result_ready=0;
        if(busy || result_valid) errors=errors+1;
        if(errors==0) $display("[TB_PASS] projection pipeline: 32 boundary cases, 1280 scores, 288 updates, 9 ties, result stall; cycles=%0d",cycles-begin_cycle);
        else $display("[TB_FAIL] projection errors=%0d",errors);
        $finish;
    end
    initial begin #1000000;$display("[TB_FAIL] projection timeout");$finish;end
endmodule
