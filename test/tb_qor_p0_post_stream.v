`timescale 1ns/1ps
// 真正运行Conv1d控制器：不force状态/数据。短层覆盖K尾、Cout尾、m边界，
// SRAM可变延迟和输出长反压；逐Byte对拍、逐Tag检查新增弹性级。
module tb_qor_p0_post_stream;
    reg clk=0,rst_n=0,start=0;
    reg [8:0] cin=1,cout=13,input_length=17,output_length=17;
    reg [4:0] kernel=7;
    reg [3:0] stride=1,dilation=1;
    reg [7:0] left_pad=3;
    reg relu=0;
    wire [11:0] k_total=cin*kernel;
    wire [9:0] k_groups=(k_total+3)>>2;
    wire [5:0] n_groups=(cout+7)>>3;
    wire busy,done,error;
    wire av,ar,asr,wv,wr,wsr,pv,pr,psr,ov;
    wire [31:0] aa0,aa1,aa2,aa3,wa,oa;
    wire [3:0] mask;
    wire [5:0] param_og;
    wire [63:0] od;
    wire [7:0] os;
    reg act_pending=0,weight_pending=0,param_pending=0;
    integer act_delay=0,weight_delay=0,param_delay=0;
    reg act_valid=0,weight_valid=0,param_valid=0;
    reg [63:0] act_payload=0,act_data=0;
    reg [255:0] weight_payload=0,weight_data=0;
    reg [511:0] bias_data=0,bias_payload=0;
    reg [255:0] mult_data=0,mult_payload=0;
    reg [63:0] shift_data=0,shift_payload=0;
    reg [31:0] lfsr=32'h3748ac29;
    integer cycles=0,errors=0,case_id=0,received_bytes=0;
    integer skid_stalls=0,skid_replace=0,fifo_full=0,fifo_push_pop=0;
    integer post_stalls=0,elu_stalls=0,output_stalls=0,groups_finished=0;
    integer i,j,offset,m_id,c_id,watchdog,case_cycles;
    reg seen [0:4095];
    reg held_skid=0,held_write=0;
    reg [319:0] skid_data_q;
    reg [8:0] skid_tag_q;
    reg [31:0] write_addr_q;
    reg [63:0] write_data_q;
    reg [7:0] write_mask_q;
    reg signed [15:0] expected_word;
    // 每173周期连续阻塞前70周期，叠加LFSR气泡，足以填满Accumulator响应FIFO。
    wire out_ready=(cycles%173>=70) && lfsr[4];
    assign ar=!act_pending && !act_valid && lfsr[0];
    assign wr=!weight_pending && !weight_valid && lfsr[1];
    assign pr=!param_pending && !param_valid && lfsr[2];
    always #10 clk=~clk;
    always @(negedge clk) if(rst_n) lfsr<={lfsr[30:0],lfsr[31]^lfsr[21]^lfsr[1]^lfsr[0]};

    conv1d_controller dut (
        .clk(clk),.rst_n(rst_n),.mesh_scan_test_en(1'b0),.start(start),
        .busy(busy),.done(done),.error(error),
        .cfg_input_base(32'h1000),.cfg_output_base(32'h4000),.cfg_weight_base(32'h8000),
        .cfg_cin(cin),.cfg_cout(cout),.cfg_kernel(kernel),.cfg_stride(stride),
        .cfg_dilation(dilation),.cfg_left_pad(left_pad),.cfg_input_length(input_length),
        .cfg_output_length(output_length),.cfg_k_total(k_total),.cfg_k_groups(k_groups),
        .cfg_n_groups(n_groups),.cfg_elu_enable(1'b0),.cfg_relu_enable(relu),
        .elu_lut_cfg_valid(1'b0),.elu_lut_cfg_ready(),.elu_lut_cfg_addr(7'd0),.elu_lut_cfg_wdata(8'd0),
        .act_rd_req_valid(av),.act_rd_req_ready(ar),.act_rd_addr0(aa0),.act_rd_addr1(aa1),
        .act_rd_addr2(aa2),.act_rd_addr3(aa3),.act_rd_mask(mask),
        .act_rd_rsp_valid(act_valid),.act_rd_rsp_ready(asr),.act_rd_data(act_data),
        .wgt_rd_req_valid(wv),.wgt_rd_req_ready(wr),.wgt_rd_addr(wa),
        .wgt_rd_rsp_valid(weight_valid),.wgt_rd_rsp_ready(wsr),.wgt_rd_data(weight_data),
        .param_rd_req_valid(pv),.param_rd_req_ready(pr),.param_rd_output_group(param_og),
        .param_rd_rsp_valid(param_valid),.param_rd_rsp_ready(psr),.param_bias_data(bias_data),
        .param_mult_data(mult_data),.param_shift_data(shift_data),.param_zero_point_data(64'd0),
        .out_wr_valid(ov),.out_wr_ready(out_ready),.out_wr_addr(oa),.out_wr_data(od),.out_wr_strb(os)
    );
    function signed [15:0] activation_value;
        input integer t,ch;
        begin activation_value=((t*107+ch*31+case_id*19)%2001)-1000;end
    endfunction
    function signed [7:0] weight_value;
        input integer ch,r;
        begin weight_value=((r*3+ch*5)%9)-4;end
    endfunction
    function signed [31:0] bias_value;
        input integer ch;
        begin bias_value=(ch%3-1)*17;end
    endfunction
    function [63:0] read_activation;
        input [31:0] a0,a1,a2,a3;
        input [3:0] mask_value;
        integer lane_id,index;
        reg [31:0] address_value;
        reg [63:0] v;
        begin
            v=0;
            for(lane_id=0;lane_id<4;lane_id=lane_id+1) begin
                case(lane_id) 0:address_value=a0;1:address_value=a1;2:address_value=a2;3:address_value=a3;endcase
                if(mask_value[lane_id]) begin
                    index=(address_value-32'h1000)/2;
                    if(index<0 || index>=input_length*cin || address_value[0]) begin
                        $display("[ERROR] invalid activation address %h",address_value);errors=errors+1;
                    end
                    v[16*lane_id+:16]=activation_value(index/cin,index%cin);
                end
            end
            read_activation=v;
        end
    endfunction
    function [255:0] read_weight;
        input [31:0] address_value;
        integer b,og,kg,row,col,r,ch;
        reg [255:0] v;
        begin
            b=(address_value-32'h8000)/32;og=b/k_groups;kg=b%k_groups;v=0;
            for(row=0;row<4;row=row+1) for(col=0;col<8;col=col+1) begin
                r=kg*4+row;ch=og*8+col;
                // 故意给尾部填非零，证明DUT确实做了K/Cout mask。
                v[8*(row*8+col)+:8]=(r<k_total && ch<cout) ? weight_value(ch,r) : 8'h7f;
            end
            read_weight=v;
        end
    endfunction
    function signed [15:0] golden;
        input integer t_out,ch_out;
        integer k,ch_in,t_in,r,mul,s;
        reg signed [63:0] acc,product,mag,value;
        begin
            acc=0;
            for(k=0;k<kernel;k=k+1) for(ch_in=0;ch_in<cin;ch_in=ch_in+1) begin
                t_in=t_out*stride+k*dilation-left_pad;r=k*cin+ch_in;
                if(t_in>=0 && t_in<input_length)
                    acc=acc+activation_value(t_in,ch_in)*weight_value(ch_out,r);
            end
            mul=(ch_out%2) ? 3 : 1;s=ch_out%4+2;
            product=(acc+bias_value(ch_out))*mul;mag=(product<0) ? -product : product;
            value=mag/(64'd1<<s);
            if(mag%(64'd1<<s)>=(64'd1<<(s-1))) value=value+1;
            if(product<0) value=-value;
            if(value>32767) value=32767;
            if(value< -32768) value=-32768;
            if(relu && value<0) value=0;
            golden=value[15:0];
        end
    endfunction

    // 单笔可变延迟SRAM响应模型。req/rsp各自握手，响应被反压时保持。
    always @(posedge clk) begin
        cycles=cycles+1;
        if(!rst_n) begin
            act_pending<=0;weight_pending<=0;param_pending<=0;
            act_valid<=0;weight_valid<=0;param_valid<=0;
        end else begin
            if(act_valid && asr) act_valid<=0;
            if(weight_valid && wsr) weight_valid<=0;
            if(param_valid && psr) param_valid<=0;
            if(act_pending) begin
                if(act_delay==0) begin act_valid<=1;act_data<=act_payload;act_pending<=0;end
                else act_delay<=act_delay-1;
            end
            if(weight_pending) begin
                if(weight_delay==0) begin weight_valid<=1;weight_data<=weight_payload;weight_pending<=0;end
                else weight_delay<=weight_delay-1;
            end
            if(param_pending) begin
                if(param_delay==0) begin
                    param_valid<=1;bias_data<=bias_payload;mult_data<=mult_payload;
                    shift_data<=shift_payload;param_pending<=0;
                end else param_delay<=param_delay-1;
            end
            if(av && ar) begin
                act_payload<=read_activation(aa0,aa1,aa2,aa3,mask);
                act_pending<=1;act_delay<=lfsr[8:6];
            end
            if(wv && wr) begin
                weight_payload<=read_weight(wa);weight_pending<=1;weight_delay<=lfsr[10:9];
            end
            if(pv && pr) begin
                for(j=0;j<8;j=j+1) begin
                    bias_payload[64*j+:64]<=$signed(bias_value(param_og*8+j));
                    mult_payload[32*j+:32]<=((param_og*8+j)%2) ? 3 : 1;
                    shift_payload[8*j+:8]<=(param_og*8+j)%4+2;
                end
                param_pending<=1;param_delay<=lfsr[12:11];
            end
        end
    end
    always @(posedge clk) begin
        if(!rst_n) begin held_skid=0;held_write=0;end
        else begin
            if(error) begin $display("[ERROR] controller error");errors=errors+1;end
            if(held_skid && (!dut.post_skid_valid || dut.post_skid_data!==skid_data_q || dut.post_skid_tag!==skid_tag_q)) begin
                $display("[ERROR] skid data/tag changed under stall");errors=errors+1;
            end
            held_skid=dut.post_skid_valid && !dut.post_in_ready;
            skid_data_q=dut.post_skid_data;skid_tag_q=dut.post_skid_tag;
            if(held_skid) skid_stalls=skid_stalls+1;
            if(dut.post_skid_valid && dut.post_in_ready && dut.acc_rd_rsp_valid && dut.acc_rd_rsp_ready)
                skid_replace=skid_replace+1;
            if(dut.u_acc.post_rsp_count==8) fifo_full=fifo_full+1;
            if(dut.u_acc.post_read_pending && dut.u_acc.post_rsp_pop) fifo_push_pop=fifo_push_pop+1;
            if(dut.post_valid && !dut.post_ready) post_stalls=post_stalls+1;
            if(dut.elu_out_valid && !dut.elu_out_ready) elu_stalls=elu_stalls+1;
            if(dut.post_valid!==dut.post_tag_valid || dut.elu_out_valid!==dut.elu_tag_valid) begin
                $display("[ERROR] post/ELU valid-tag mismatch");errors=errors+1;
            end
            if(held_write && (!ov || oa!==write_addr_q || od!==write_data_q || os!==write_mask_q)) begin
                $display("[ERROR] output changed under stall");errors=errors+1;
            end
            held_write=ov && !out_ready;
            write_addr_q=oa;write_data_q=od;write_mask_q=os;
            if(held_write) output_stalls=output_stalls+1;
            if(dut.writer_vector_done) groups_finished=groups_finished+1;
            if(ov && out_ready) for(i=0;i<8;i=i+1) if(os[i]) begin
                offset=oa-32'h4000+i;
                if(offset<0 || offset>=output_length*cout*2) begin
                    $display("[ERROR] output address out of range %h",oa+i);errors=errors+1;
                end else begin
                    m_id=(offset/2)/cout;c_id=(offset/2)%cout;expected_word=golden(m_id,c_id);
                    if(seen[offset] || od[i*8+:8] !== ((offset%2) ? expected_word[15:8] : expected_word[7:0])) begin
                        $display("[ERROR] case=%0d m=%0d c=%0d byte=%0d data=%h expected16=%h duplicate=%0d",case_id,m_id,c_id,offset%2,od[i*8+:8],expected_word,seen[offset]);
                        errors=errors+1;
                    end
                    seen[offset]=1;received_bytes=received_bytes+1;
                end
            end
        end
    end
    task run_case;
        input integer id;
        integer n;
        begin
            @(negedge clk);case_id=id;received_bytes=0;
            for(n=0;n<4096;n=n+1) seen[n]=0;
            case(id)
                0:begin cin=1;cout=13;kernel=7;input_length=17;output_length=17;left_pad=3;relu=0;end
                1:begin cin=8;cout=8;kernel=1;input_length=19;output_length=19;left_pad=0;relu=1;end
                2:begin cin=1;cout=1;kernel=1;input_length=1;output_length=1;left_pad=0;relu=0;end
                3:begin cin=8;cout=9;kernel=3;input_length=9;output_length=9;left_pad=1;relu=0;end
            endcase
            case_cycles=cycles;start=1;
            @(negedge clk);start=0;watchdog=0;
            while(!done && !error && watchdog<20000) begin @(negedge clk);watchdog=watchdog+1;end
            if(!done || received_bytes!=output_length*cout*2) begin
                $display("[ERROR] case=%0d done=%b bytes=%0d/%0d",id,done,received_bytes,output_length*cout*2);errors=errors+1;
            end
            for(n=0;n<output_length*cout*2;n=n+1) if(!seen[n]) errors=errors+1;
            $display("[CASE_DONE] case=%0d cycles=%0d bytes=%0d",id,cycles-case_cycles,received_bytes);
            repeat(5) @(negedge clk);
            if(dut.post_skid_valid || dut.post_tag_valid || dut.elu_tag_valid || dut.u_acc.post_rsp_count!=0) begin
                $display("[ERROR] pipeline not empty at layer boundary");errors=errors+1;
            end
        end
    endtask
    initial begin
        repeat(5) @(negedge clk);rst_n=1;
        run_case(0);run_case(1);run_case(2);run_case(3);
        if(skid_stalls==0 || skid_replace==0 || fifo_full==0 || fifo_push_pop==0 ||
           post_stalls==0 || elu_stalls==0 || output_stalls==0) begin
            $display("[ERROR] insufficient stall/FIFO coverage");errors=errors+1;
        end
        $display("[POST_COVERAGE] skid_stall=%0d replace=%0d fifo_full=%0d simultaneous_push_pop=%0d post_stall=%0d elu_stall=%0d output_stall=%0d vectors=%0d",skid_stalls,skid_replace,fifo_full,fifo_push_pop,post_stalls,elu_stalls,output_stalls,groups_finished);
        if(errors==0) $display("[TB_PASS] real Conv1d + accumulator + elastic data/tag + postprocess + ELU + writer under random/long stalls");
        else $display("[TB_FAIL] post stream errors=%0d",errors);
        $finish;
    end
    initial begin #3000000;$display("[TB_FAIL] post stream timeout");$finish;end
endmodule
