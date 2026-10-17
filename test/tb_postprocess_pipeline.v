`timescale 1ns/1ps
// 独立8路整数参考（保留旧80-bit算法，不调用DUT函数）+ 事务顺序计分板。
// 覆盖完整UINT8 shift、INT40/INT32极值、正负半值、逐拍换参、三层反压和复位冲刷。
module tb_postprocess_pipeline;
    reg clk=0, rst_n=0, in_valid=0, post_ready=0;
    wire in_ready, post_valid;
    reg [319:0] acc=0;
    reg [511:0] bias=0;
    reg [255:0] mult=0;
    reg [63:0] shift=0, zp=0;
    reg [15:0] tag=0;
    wire [127:0] data;
    wire [15:0] out_tag;
    integer errors=0, cycles=0, sent=0, received=0, lane, k, watchdog;
    integer max_burst=0, burst=0, stalled0=0, stalled1=0, stalled2=0, full=0;
    integer seed=32'h29871abc;
    reg [31:0] random_bits=32'h1ac27349;
    reg input_fired=0, held0=0, held1=0, held2=0;
    reg [719:0] held_data0, held_data1;
    reg [143:0] held_data2;
    reg [127:0] expected[0:2047];
    reg [15:0] expected_tag[0:2047];
    integer accept_cycle[0:2047];
    reg shift_seen[0:255];
    reg signed [39:0] av,bv;
    reg signed [31:0] mv;
    reg signed [7:0] zv;
    reg [7:0] sv;
    wire [719:0] bundle0={dut.pp0_biased,dut.pp0_mult,dut.pp0_shift,dut.pp0_zp,dut.pp0_tag};
    wire [719:0] bundle1={dut.pp1_product,dut.pp1_shift,dut.pp1_zp,dut.pp1_tag};
    wire [143:0] bundle2={data,out_tag};
    always #10 clk=~clk;
    postprocess_8lane #(.TAG_WIDTH(16)) dut(
        .clk(clk),.rst_n(rst_n),.in_valid(in_valid),.in_ready(in_ready),
        .acc_data(acc),.bias_data(bias),.mult_data(mult),.shift_data(shift),
        .zero_point_data(zp),.post_valid(post_valid),.post_ready(post_ready),
        .post_data(data),.in_tag(tag),.post_tag(out_tag)
    );

    // 按修改前代码的运算顺序和80位溢出语义独立求值，特别包含shift=80。
    function [15:0] legacy_reference;
        input signed [39:0] a,b;
        input signed [31:0] m;
        input [7:0] s;
        input signed [7:0] z;
        reg signed [40:0] sum;
        reg signed [39:0] limited;
        reg signed [79:0] p,r,mag,v,q;
        begin
            sum={a[39],a}+{b[39],b};
            if(sum>41'sd549755813887) limited=40'sh7fffffffff;
            else if(sum< -41'sd549755813888) limited=40'sh8000000000;
            else limited=sum[39:0];
            p=$signed(limited)*$signed(m);
            if(s==0) v=p;
            else if(p>=0) begin r=p+(80'sd1<<(s-1'b1));v=r>>>s;end
            else begin mag=-p;r=mag+(80'sd1<<(s-1'b1));v=-(r>>>s);end
            q=v+{{72{z[7]}},z};
            if(q>32767) legacy_reference=16'h7fff;
            else if(q< -32768) legacy_reference=16'h8000;
            else legacy_reference=q[15:0];
        end
    endfunction

    task make_input;
        input integer id;
        integer n;
        begin
            tag=id;
            for(n=0;n<8;n=n+1) begin
                av={$random(seed),$random(seed)};
                bv={$random(seed),$random(seed)};
                mv=$random(seed);zv=$random(seed);sv=(id+n*37)%256;
                // 边界组合反复遍历所有shift；额外穿插小数半值和INT16饱和边界。
                case(id%16)
                    0:begin av=40'sh8000000000;bv=0;mv=32'sh80000000;end
                    1:begin av=40'sh7fffffffff;bv=40'sh7fffffffff;mv=32'sh7fffffff;end
                    2:begin av=40'sh8000000000;bv=40'sh8000000000;mv=32'sh7fffffff;end
                    3:begin av=0;bv=0;mv=0;end
                    4:begin av=3;bv=0;mv=1;sv=n;zv=0;end
                    5:begin av=-3;bv=0;mv=1;sv=n;zv=0;end
                    6:begin av=-4;bv=0;mv=1;sv=1;zv=0;end
                    7:begin av=32767+n;bv=0;mv=1;sv=0;zv=n;end
                    8:begin av=-32768-n;bv=0;mv=1;sv=0;zv=-n;end
                    9:begin av=40'sh8000000000;bv=0;mv=32'sh80000000;sv=71;zv=0;end
                endcase
                acc[40*n+:40]=av;
                // 高24位故意不做符号扩展，验证仍然只使用Bias容器低40位。
                bias[64*n+:64]={24'hab127e,bv};
                mult[32*n+:32]=mv;shift[8*n+:8]=sv;zp[8*n+:8]=zv;
            end
        end
    endtask

    always @(posedge clk) begin
        cycles=cycles+1;
        if(!rst_n) begin
            sent=0;received=0;held0=0;held1=0;held2=0;input_fired=0;burst=0;
        end else begin
            if(held0 && (!dut.pp0_valid || bundle0!==held_data0)) errors=errors+1;
            if(held1 && (!dut.pp1_valid || bundle1!==held_data1)) errors=errors+1;
            if(held2 && (!post_valid || bundle2!==held_data2)) errors=errors+1;
            held0=dut.pp0_valid && !dut.pp1_ready;held_data0=bundle0;
            held1=dut.pp1_valid && !dut.pp2_ready;held_data1=bundle1;
            held2=post_valid && !post_ready;held_data2=bundle2;
            if(held0) stalled0=stalled0+1;
            if(held1) stalled1=stalled1+1;
            if(held2) stalled2=stalled2+1;
            if(dut.pp0_valid && dut.pp1_valid && post_valid) full=full+1;
            input_fired=in_valid && in_ready;
            if(input_fired) begin
                for(lane=0;lane<8;lane=lane+1) begin
                    expected[sent][16*lane+:16]=legacy_reference(
                        $signed(acc[40*lane+:40]),$signed(bias[64*lane+:40]),
                        $signed(mult[32*lane+:32]),shift[8*lane+:8],$signed(zp[8*lane+:8]));
                    shift_seen[shift[8*lane+:8]]=1;
                end
                expected_tag[sent]=tag;accept_cycle[sent]=cycles;sent=sent+1;
            end
            if(post_valid && post_ready) begin
                if(received>=sent || data!==expected[received] || out_tag!==expected_tag[received]) begin
                    $display("[ERROR] PP vector=%0d tag=%h expected_tag=%h actual=%h expected=%h",received,out_tag,expected_tag[received],data,expected[received]);errors=errors+1;
                end
                if(cycles-accept_cycle[received]<3) errors=errors+1;
                if(received<60 && cycles-accept_cycle[received]!=3) errors=errors+1;
                received=received+1;burst=burst+1;
                if(burst>max_burst) max_burst=burst;
            end else burst=0;
        end
    end

    initial begin
        for(k=0;k<256;k=k+1) shift_seen[k]=0;
        repeat(4) @(negedge clk);rst_n=1;
        // 先放入一笔，再在PP0/PP1仍有数据时复位，检查旧事务不泄漏到下一轮。
        make_input(0);in_valid=1;post_ready=0;
        repeat(2) @(negedge clk);rst_n=0;in_valid=0;
        repeat(2) @(negedge clk);
        if(post_valid!==0 || dut.pp0_valid!==0 || dut.pp1_valid!==0) errors=errors+1;
        rst_n=1;
        while(received<1024 && cycles<20000) begin
            // 第一段无气泡证明II=1；之后加入50拍长反压与随机输入/输出气泡。
            post_ready=(sent<80) || ((cycles%97>=50) && random_bits[2]);
            if(!in_valid || input_fired) begin
                in_valid=(sent<1024) && ((sent<80) || random_bits[0]);
                if(in_valid) make_input(sent);
            end
            random_bits={random_bits[30:0],random_bits[31]^random_bits[21]^random_bits[1]^random_bits[0]};
            @(negedge clk);
        end
        in_valid=0;post_ready=1;
        repeat(5) @(negedge clk);
        for(k=0;k<256;k=k+1) if(!shift_seen[k]) errors=errors+1;
        if(sent!=1024 || received!=1024 || max_burst<32 || full==0 ||
           stalled0==0 || stalled1==0 || stalled2==0 || post_valid || dut.pp0_valid || dut.pp1_valid) errors=errors+1;
        $display("[PP_COVERAGE] vectors=%0d lanes=%0d shifts=256 burst=%0d full=%0d stalls=%0d/%0d/%0d",received,received*8,max_burst,full,stalled0,stalled1,stalled2);
        if(errors==0) $display("[TB_PASS] PP three-stage bit-exact legacy rounding + tags + II=1 + stalls + reset flush");
        else $display("[TB_FAIL] PP pipeline errors=%0d sent=%0d received=%0d",errors,sent,received);
        $finish;
    end
    initial begin #500000; $display("[TB_FAIL] PP timeout"); $finish;end
endmodule
