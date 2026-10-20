`timescale 1ns/1ps
// 独立旧64/80位公式对拍：不能调用优化后的Guard算法生成Golden。
module tb_rvq_backend_arithmetic;
    reg clk=0;
    always #10 clk=~clk;
    rvq_backend_w8a16 #(.COMPUTE_VECTOR(0)) dut(
        .clk(clk),.rst_n(1'b0),.boot_write(1'b0),.boot_index(16'd0),.boot_data(256'd0),
        .start(1'b0),.latent_base(32'd0),.latent_req_ready(1'b0),
        .latent_rsp_valid(1'b0),.latent_data(64'd0),.result_ready(1'b0)
    );
    integer errors=0,checks49=0,checks72=0,checks_vector=0;
    integer v_id,m_id,s_id,i,k,seed=32'h7c89a423;
    reg signed [16:0] v17;
    reg signed [39:0] v40;
    reg signed [31:0] m32;
    reg signed [48:0] p49;
    reg signed [71:0] p72;
    reg signed [39:0] pv40;
    reg [7:0] shift;
    reg signed [79:0] boundary;

    function signed [15:0] sat16;
        input signed [79:0] p;
        begin
            if(p>32767) sat16=16'h7fff;
            else if(p< -32768) sat16=16'h8000;
            else sat16=p[15:0];
        end
    endfunction
    function signed [63:0] old_rq64;
        input signed [16:0] v;
        input signed [31:0] m;
        input [7:0] s;
        reg signed [63:0] p;
        reg [63:0] mag,q;
        begin
            p=v*m;mag=(p<0)?-p:p;q=mag>>s;
            if(s!=0 && (mag & ((64'd1<<s)-1)) >= (64'd1<<(s-1))) q=q+1;
            old_rq64=(p<0)?-$signed(q):$signed(q);
        end
    endfunction
    function signed [79:0] old_round80;
        input signed [79:0] p;
        input [7:0] s;
        reg [79:0] mag,q;
        begin
            mag=(p<0)?-p:p;
            q=(s==0)?mag:(mag+(80'd1<<(s-1)))>>s;
            old_round80=(p<0)?-$signed(q):$signed(q);
        end
    endfunction
    task check49;
        input signed [16:0] v;
        input signed [31:0] m;
        input [7:0] s;
        reg signed [48:0] p;
        begin
            p=$signed(v)*$signed(m);
            checks49=checks49+1;
            if(dut.round_sat16_49(p,s)!==sat16(old_rq64(v,m,s))) begin
                if(errors<10) $display("[ERROR] rq49 v=%0d m=%0d s=%0d",v,m,s);
                errors=errors+1;
            end
        end
    endtask
    task check72;
        input signed [71:0] p;
        input [7:0] s;
        begin
            checks72=checks72+1;
            if(dut.round_sat16_72(p,s)!==sat16(old_round80(p,s))) begin
                if(errors<10) $display("[ERROR] projection72 p=%h s=%0d",p,s);
                errors=errors+1;
            end
        end
    endtask
    task check_vector;
        input signed [7:0] v;
        input signed [31:0] m;
        input [7:0] s;
        reg signed [39:0] p;
        reg signed [48:0] got;
        begin
            p=$signed(v)*$signed(m);got=dut.round_vector49(p,s);
            checks_vector=checks_vector+1;
            if($signed(got)!==old_rq64(v,m,s)) begin
                if(errors<10) $display("[ERROR] vector49 v=%0d m=%0d s=%0d",v,m,s);
                errors=errors+1;
            end
        end
    endtask

    initial begin
        #1;
        // Shift完整UINT8域；包括非法参数旧行为，不借位宽优化偷偷改变异常语义。
        for(s_id=0;s_id<256;s_id=s_id+1)
            for(v_id=0;v_id<9;v_id=v_id+1)
                for(m_id=0;m_id<7;m_id=m_id+1) begin
                    case(v_id)
                        0:v17=-65536;1:v17=65535;2:v17=-32895;3:v17=32895;
                        4:v17=0;5:v17=1;6:v17=-1;7:v17=32767;8:v17=-32768;
                    endcase
                    case(m_id)
                        0:m32=32'h80000000;1:m32=32'h7fffffff;2:m32=0;
                        3:m32=1;4:m32=-1;5:m32=65535;6:m32=-65535;
                    endcase
                    check49(v17,m32,s_id);
                    check_vector(v17[7:0],m32,s_id);
                    v40=(v_id%2)?40'sh7fffffffff:40'sh8000000000;
                    p72=$signed(v40)*$signed(m32);check72(p72,s_id);
                end
        // 穷举17位输入，特别覆盖(-65536)*(-2^31)=+2^47，48位signed不可容纳。
        for(i=-65536;i<=65535;i=i+1) begin
            check49(i,1,0);check49(i,-1,1);
            check49(i,32'h80000000,32);
            check49(i,32'h80000000,48);
            check49(i,32'h7fffffff,49);
        end
        // 饱和边界及正负半值：舍入前后32767/32768/32769。
        for(s_id=1;s_id<=55;s_id=s_id+1)
            for(k=32766;k<=32769;k=k+1) begin
                boundary=(80'd1<<s_id)*k+(80'd1<<(s_id-1));
                check72(boundary-1,s_id);check72(boundary,s_id);check72(boundary+1,s_id);
                check72(-boundary+1,s_id);check72(-boundary,s_id);check72(-boundary-1,s_id);
            end
        for(i=0;i<50000;i=i+1) begin
            v17=$random(seed);m32=$random(seed);shift=$random(seed);
            check49(v17,m32,shift);check_vector(v17[7:0],m32,shift);
            v40={$random(seed),$random(seed)};
            p72=$signed(v40)*$signed(m32);check72(p72,shift);
        end
        $display("[RVQ_ARITH] rq49=%0d projection72=%0d vector49=%0d",checks49,checks72,checks_vector);
        if(errors==0) $display("[TB_PASS] RVQ49 and projection72 Guard saturation match legacy formulas, vector remains unsaturated");
        else $display("[TB_FAIL] RVQ arithmetic errors=%0d",errors);
        $finish;
    end
endmodule
