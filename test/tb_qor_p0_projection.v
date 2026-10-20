`timescale 1ns/1ps
// 投影分拍专项：真实SRAM接口装载合成参数，不force DUT内部状态。
// 覆盖INT40加Bias饱和、正负乘数、shift=0/1/63、正负半值舍入和INT16饱和。
module tb_qor_p0_projection #(parameter SCENARIO=0);
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
    integer update_mul_states=0,update_rnd_states=0,reset_cases=0;
    reg checking=0,expected_error=0;
    reg signed [15:0] golden_residual[0:31],golden_next[0:287],golden_before[0:287];
    reg signed [7:0] golden_code[0:287];
    reg signed [16:0] golden_sub[0:287];
    reg signed [31:0] golden_score[0:1279],golden_sum[0:31];
    reg [7:0] golden_index[0:8];
    reg [63:0] golden_indices;
    reg [511:0] golden_vector;
    reg [511:0] held_vector;
    reg [4:0] mul_dimension;
    reg previous_mul=0;
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
    function signed [7:0] book_value;
        input integer entry_id,dim_id;
        begin
            if(SCENARIO==0) book_value=0;
            else if(entry_id%17==0) book_value=(dim_id%2)?127:-128;
            else book_value=(entry_id%11-5)*(dim_id%5-2)+(entry_id/128)%3-1;
        end
    endfunction
    function signed [31:0] next_multiplier;
        input integer st;
        begin
            if(SCENARIO==0) next_multiplier=1;
            else case(st%4) 0:next_multiplier=3;1:next_multiplier=-3;
                2:next_multiplier=1;3:next_multiplier=2;endcase
        end
    endfunction
    function [7:0] next_shift;
        input integer st;
        begin next_shift=(SCENARIO==0)?0:st%3;end
    endfunction
    function signed [31:0] vector_multiplier;
        input integer st;
        begin
            if(SCENARIO==0) vector_multiplier=1;
            else if(SCENARIO==2) vector_multiplier=32'h7fffffff;
            else vector_multiplier=(st%2)?-65536:65536;
        end
    endfunction
    function signed [63:0] quant_reference;
        input signed [16:0] v;
        input signed [31:0] m;
        input [7:0] s;
        reg signed [63:0] p,mag;
        begin
            p=v*m;mag=(p<0)?-p:p;
            if(s!=0) mag=(mag+(64'sd1<<(s-1)))>>>s;
            quant_reference=(p<0)?-mag:mag;
        end
    endfunction
    function signed [15:0] clamp16;
        input signed [63:0] v;
        begin
            if(v>32767) clamp16=32767;
            else if(v< -32768) clamp16=16'h8000;
            else clamp16=v[15:0];
        end
    endfunction
    // 独立整级搜索，计算每候选score、最小索引、每维残差和最终重建值。
    task build_golden;
        integer st,d,e,start_entry,count,si,winner,ui;
        reg signed [31:0] score,norm_value,best;
        reg signed [63:0] dot,sum;
        reg signed [16:0] diff;
        begin
            golden_indices=0;golden_vector=0;si=0;
            for(d=0;d<32;d=d+1) begin golden_residual[d]=expected[d];golden_sum[d]=0;end
            for(st=0;st<9;st=st+1) begin
                start_entry=(st==0)?0:256+(st-1)*128;count=(st==0)?256:128;
                best=32'h7fffffff;winner=0;
                for(e=0;e<count;e=e+1) begin
                    dot=0;norm_value=0;
                    for(d=0;d<32;d=d+1) begin
                        dot=dot+$signed(golden_residual[d])*$signed(book_value(start_entry+e,d));
                        norm_value=norm_value+$signed(book_value(start_entry+e,d))*$signed(book_value(start_entry+e,d));
                    end
                    score=norm_value-(dot*2);golden_score[si]=score;si=si+1;
                    if(score<best) begin best=score;winner=e;end
                end
                golden_index[st]=winner;
                if(st==0) golden_indices[7:0]=winner;
                else golden_indices[8+(st-1)*7+:7]=winner;
                for(d=0;d<32;d=d+1) begin
                    ui=st*32+d;golden_before[ui]=golden_residual[d];
                    golden_code[ui]=book_value(start_entry+winner,d);
                    diff=$signed(golden_residual[d])-$signed(golden_code[ui]);
                    golden_sub[ui]=diff;
                    golden_next[ui]=clamp16(quant_reference(diff,next_multiplier(st),next_shift(st)));
                    if(st!=8) golden_residual[d]=golden_next[ui];
                    sum=$signed(golden_sum[d])+quant_reference(golden_code[ui],vector_multiplier(st),0);
                    if(sum>2147483647 || sum< -64'sd2147483648) expected_error=1;
                    golden_sum[d]=sum[31:0];
                end
            end
            for(d=0;d<32;d=d+1) golden_vector[d*16+:16]=clamp16(golden_sum[d]);
        end
    endtask
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
        integer lane_id,ch,d,entry_id;
        reg signed [31:0] norm_value;
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
            end else if(id>=80 && id<1360) begin
                for(d=0;d<32;d=d+1) v[d*8+:8]=book_value(id-80,d);
            end else if(id>=1360 && id<1520) begin
                for(lane_id=0;lane_id<8;lane_id=lane_id+1) begin
                    entry_id=(id-1360)*8+lane_id;norm_value=0;
                    for(d=0;d<32;d=d+1)
                        norm_value=norm_value+$signed(book_value(entry_id,d))*$signed(book_value(entry_id,d));
                    v[lane_id*32+:32]=norm_value;
                end
            end else if(id>=1520 && id<1529) begin
                v[31:0]=next_multiplier(id-1520);v[39:32]=next_shift(id-1520);
                v[71:40]=vector_multiplier(id-1520);v[79:72]=0;
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
            if(checking) begin
            if(dut.state==23) bias_states=bias_states+1;
            if(dut.state==24) mul_states=mul_states+1;
            if(dut.state==8) quant_states=quant_states+1;
            if(dut.state==19) begin
                update_mul_states=update_mul_states+1;mul_dimension=dut.dimension;
            end
            if(dut.state==25) begin
                update_rnd_states=update_rnd_states+1;
                if(!previous_mul || dut.dimension!==mul_dimension || dut.update_dimension_q!==mul_dimension)
                    begin $display("[ERROR] UPDATE dimension advanced before writeback");errors=errors+1;end
            end
            previous_mul=(dut.state==19);
            if(error && !expected_error) begin $display("[ERROR] RVQ error asserted");errors=errors+1;end
            if(projection_valid) begin
                if(projection_seen[projection_channel] || projection_data!==expected[projection_channel]) begin
                    $display("[ERROR] projection c=%0d expected=%0d actual=%0d",projection_channel,expected[projection_channel],projection_data);
                    errors=errors+1;
                end
                projection_seen[projection_channel]=1;
                projections=projections+1;
            end
            if(score_valid) begin
                if(trace_score!==golden_score[scores] || trace_stage!==((scores<256)?0:1+(scores-256)/128) ||
                   trace_candidate!==((scores<256)?scores:(scores-256)%128))
                    begin $display("[ERROR] score/order mismatch at %0d",scores);errors=errors+1;end
                scores=scores+1;
            end
            if(update_valid) begin
                if(trace_stage!==(updates/32) || trace_dimension!==(updates%32) ||
                   trace_residual!==golden_before[updates] || trace_codeword!==golden_code[updates] ||
                   trace_subtract!==golden_sub[updates] || trace_next!==golden_next[updates]) begin
                    $display("[ERROR] residual update mismatch stage=%0d dim=%0d",trace_stage,trace_dimension);
                    errors=errors+1;
                end
                updates=updates+1;
            end
            if(stage_valid) begin
                if(trace_stage!==stages || trace_index!==golden_index[stages]) begin
                    $display("[ERROR] best index mismatch stage=%0d",stages);errors=errors+1;end
                stages=stages+1;
            end
            end // checking
        end
    end
    task boot_backend;
        begin
            for(block_id=0;block_id<1536;block_id=block_id+1) begin
                @(negedge clk);boot_write=1;boot_index=block_id;boot_data=image_block(block_id);
            end
            @(negedge clk);boot_write=0;
            if(!configured) begin $display("[TB_FAIL] boot incomplete");$finish;end
        end
    endtask
    initial begin
        for(c=0;c<32;c=c+1) expected[c]=golden_projection(c);
        build_golden;
        repeat(4) @(negedge clk);
        rst_n=1;
        // 非零场景分别在乘法级/舍入级中途复位，再重新BOOT执行完整流程。
        if(SCENARIO==1) begin
            for(c=0;c<2;c=c+1) begin
                boot_backend;start=1;@(negedge clk);start=0;
                wait(dut.state==((c==0)?19:25));@(negedge clk);
                rst_n=0;repeat(3) @(negedge clk);
                if(busy || result_valid || update_valid || configured) errors=errors+1;
                rst_n=1;reset_cases=reset_cases+1;
            end
        end
        boot_backend;checking=1;
        start=1;begin_cycle=cycles;
        @(negedge clk);start=0;
        wait(result_valid);
        @(negedge clk);held_indices=result_indices;held_vector=result_vector;
        repeat(13) begin
            @(negedge clk);
            if(!result_valid || result_indices!==held_indices || result_vector!==held_vector) errors=errors+1;
        end
        if(result_indices!==golden_indices || result_vector!==golden_vector || error!==expected_error ||
           projections!=32 || scores!=1280 || updates!=288 || stages!=9 ||
           bias_states!=32 || mul_states!=32 || quant_states!=32 ||
           update_mul_states!=288 || update_rnd_states!=288 || (SCENARIO==1 && reset_cases!=2)) begin
            $display("[ERROR] coverage projections=%0d scores=%0d updates=%0d stages=%0d bias/mul/quant=%0d/%0d/%0d",projections,scores,updates,stages,bias_states,mul_states,quant_states);
            errors=errors+1;
        end
        result_ready=1;
        @(negedge clk);result_ready=0;
        if(busy || result_valid) errors=errors+1;
        $display("[RVQ_UPDATE_COVERAGE] scenario=%0d mul=%0d rnd=%0d resets=%0d overflow_expected=%0d",SCENARIO,update_mul_states,update_rnd_states,reset_cases,expected_error);
        if(SCENARIO==2 && !expected_error) errors=errors+1;
        if(errors==0) $display("[TB_PASS] backend scenario=%0d: 32 projections, 1280 scores, 288 aligned updates, 9 best indices, vector and stalls; cycles=%0d",SCENARIO,cycles-begin_cycle);
        else $display("[TB_FAIL] projection errors=%0d",errors);
        $finish;
    end
    initial begin #2000000;$display("[TB_FAIL] projection timeout");$finish;end
endmodule

module tb_rvq_backend_update;
    tb_qor_p0_projection #(.SCENARIO(1)) u_test();
endmodule
module tb_rvq_backend_vector_overflow;
    tb_qor_p0_projection #(.SCENARIO(2)) u_test();
endmodule
