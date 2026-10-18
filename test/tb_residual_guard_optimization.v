`timescale 1ns/1ps
// 算术独立64位Golden + 四状态独立参考逐周期A/B + 事务计分板。
// 0/1为优化后旁路/Scale对齐，2/3为相同四状态、旧64位算术的参考模型。
module tb_residual_guard_optimization;
    reg clk=0,rst_n=0,in_valid=0,out_ready=0,scratch_ready=0,rsp_valid=0;
    reg [31:0] addr=32'h4000,base=32'h4000;
    reg [63:0] payload=0,rsp_data=0;
    reg [7:0] strb=0;
    reg [119:0] params=0;
    wire [3:0] ready,busy,rd_valid,ov,error;
    wire [10:0] rd_addr[0:3];
    wire [31:0] oa[0:3];
    wire [63:0] od[0:3];
    wire [7:0] os[0:3];
    genvar g;
    generate for(g=0;g<4;g=g+1) begin:g_unit
        if(g<2) begin:g_new
            residual_add_8lane #(.SCALE_ALIGN_ENABLE(g%2)) dut(
                .clk(clk),.rst_n(rst_n),.in_valid(in_valid),.in_ready(ready[g]),.busy(busy[g]),
                .in_addr(addr),.output_base(base),.in_data(payload),.in_strb(strb),.scale_params(params),
                .scratch_rd_valid(rd_valid[g]),.scratch_rd_ready(scratch_ready),.scratch_rd_addr(rd_addr[g]),
                .scratch_rsp_valid(rsp_valid),.scratch_rsp_data(rsp_data),.out_valid(ov[g]),.out_ready(out_ready),
                .out_addr(oa[g]),.out_data(od[g]),.out_strb(os[g]),.error(error[g])
            );
        end else begin:g_old
            residual_add_8lane_pipeline_reference #(.SCALE_ALIGN_ENABLE(g%2)) dut(
                .clk(clk),.rst_n(rst_n),.in_valid(in_valid),.in_ready(ready[g]),.busy(busy[g]),
                .in_addr(addr),.output_base(base),.in_data(payload),.in_strb(strb),.scale_params(params),
                .scratch_rd_valid(rd_valid[g]),.scratch_rd_ready(scratch_ready),.scratch_rd_addr(rd_addr[g]),
                .scratch_rsp_valid(rsp_valid),.scratch_rsp_data(rsp_data),.out_valid(ov[g]),.out_ready(out_ready),
                .out_addr(oa[g]),.out_data(od[g]),.out_strb(os[g]),.error(error[g])
            );
        end
    end endgenerate

    integer errors=0,arith_checks=0,sum_checks=0,seed=32'h61ab9832;
    integer s_idx,v_idx,m_idx,j,k,z_idx,cycles=0,sent=0,received=0,requests=0,responses=0;
    integer input_stalls=0,read_stalls=0,output_stalls=0,overlap=0,reset_flush=0,bad_addresses=0;
    integer delay_left=0,completed_beats=0;
    integer wait2_checks=0;
    reg expect_wait2=0,expect_output=0;
    reg signed [15:0] v,identity;
    reg signed [31:0] multiplier;
    reg [7:0] shift;
    reg signed [63:0] wide,expected_clip,wide_sum;
    reg signed [15:0] got16;
    reg signed [16:0] got17;
    reg [31:0] lfsr=32'hc78124ab;
    reg input_fire=0,pending=0,held_out=0,held_read=0;
    reg [63:0] pending_data=0;
    reg [103:0] held_bundle0,held_bundle1;
    reg [10:0] held_read_addr;
    reg [63:0] expected0[0:2047],expected1[0:2047];
    reg [31:0] expected_addr[0:2047];
    reg [7:0] expected_strb[0:2047];
    reg strb_seen[0:255];
    reg [63:0] identity_word;
    reg signed [15:0] ialigned,maligned;
    reg [103:0] bundle0,bundle1;
    always #10 clk=~clk;

    function signed [63:0] ref_requant;
        input signed [15:0] value;
        input signed [31:0] m;
        input [7:0] s;
        reg signed [63:0] product,mag;
        begin
            product=$signed(value)*$signed(m);
            if(s==0) ref_requant=product;
            else if(s>=64) ref_requant=0;
            else begin
                mag=(product<0)?-product:product;
                mag=(mag+(64'sd1<<(s-1)))>>>s;
                ref_requant=(product<0)?-mag:mag;
            end
        end
    endfunction
    function [15:0] ref_sat16;
        input signed [63:0] value;
        begin
            if(value>32767) ref_sat16=16'h7fff;
            else if(value< -32768) ref_sat16=16'h8000;
            else ref_sat16=value[15:0];
        end
    endfunction

    task check_arithmetic;
        input signed [15:0] value;
        input signed [31:0] m;
        input [7:0] s;
        integer n;
        begin
            wide=ref_requant(value,m,s);
            got16=g_unit[1].g_new.dut.requant_sat16(value,m,s);
            got17=g_unit[1].g_new.dut.requant_clip17(value,m,s);
            if(wide>65535) expected_clip=65535;
            else if(wide< -65535) expected_clip=-65535;
            else expected_clip=wide;
            arith_checks=arith_checks+1;
            if(got16!==ref_sat16(wide) || $signed(got17)!==expected_clip) begin
                if(errors<12) $display("[ERROR] residual arithmetic v=%0d m=%0d s=%0d q64=%0d got16=%0d got17=%0d",value,m,s,wide,got16,got17);
                errors=errors+1;
            end
            for(n=0;n<5;n=n+1) begin
                case(n) 0:identity=-32768;1:identity=32767;2:identity=-1;3:identity=0;4:identity=1;endcase
                wide_sum=$signed(identity)+$signed(got17);
                if(ref_sat16(wide_sum)!==ref_sat16($signed(identity)+wide)) errors=errors+1;
                sum_checks=sum_checks+1;
            end
        end
    endtask

    function [63:0] scratch_word;
        input [10:0] word_address;
        integer n;
        reg signed [15:0] x;
        reg [63:0] result;
        begin
            result=0;
            for(n=0;n<4;n=n+1) begin
                case((word_address+n)%8)
                    0:x=-32768;1:x=32767;2:x=-1;3:x=0;
                    4:x=1;5:x=16384;6:x=-16384;7:x=(word_address*137+n*71)%65536-32768;
                endcase
                result[16*n+:16]=x;
            end
            scratch_word=result;
        end
    endfunction

    task make_input;
        input integer id;
        integer n;
        reg signed [31:0] m1,m2,m3;
        reg [7:0] s1,s2,s3;
        begin
            base=(id%2)?32'h5000:32'h4000;
            addr=base+(id%2048)*8;strb=id;
            if(id>=1024) begin
                if(id%2) addr=base-8;
                else addr=addr+1;
            end
            for(n=0;n<4;n=n+1) begin
                case((id+n)%8)
                    0:payload[16*n+:16]=16'h8000;1:payload[16*n+:16]=16'h7fff;
                    2:payload[16*n+:16]=-3;3:payload[16*n+:16]=1;
                    4:payload[16*n+:16]=-1;5:payload[16*n+:16]=3;
                    default:payload[16*n+:16]=$random(seed);
                endcase
            end
            m1=1;m2=1;m3=1;s1=0;s2=0;s3=0;
            case(id%16)
                1:m3=65534;2:m3=65535;3:m3=65536;
                4:m3=-65534;5:m3=-65535;6:m3=-65536;
                7:begin m1=32'sh80000000;m2=32'sh80000000;m3=32'sh80000000;s1=47;s2=47;s3=47;end
                8:begin s1=1;s2=1;s3=1;end
                9:begin m1=$random(seed);m2=$random(seed);m3=$random(seed);end
                10:begin m1=32'sh7fffffff;m2=32'sh7fffffff;m3=32'sh7fffffff;s1=36;s2=36;s3=36;end
                11:begin m1=0;m2=0;m3=32'sh80000000;end
                12:begin s1=63;s2=63;s3=63;end
                13:begin s1=64;s2=64;s3=64;end
                14:begin s1=255;s2=255;s3=255;end
                15:begin m1=$random(seed);m2=$random(seed);m3=$random(seed);s1=lfsr[5:0];s2=lfsr[11:6];s3=lfsr[17:12];end
            endcase
            params={s3,m3,s2,m2,s1,m1};
        end
    endtask

    // 单笔SRAM延迟响应模型，所有候选/参考实例接收同一个合法响应。
    always @(posedge clk) begin
        cycles=cycles+1;
        if(!rst_n) begin pending<=0;rsp_valid<=0;end
        else begin
            rsp_valid<=0;
            // 无效拍也扰动返回数据，检验S_WAIT2使用锁存值而不是SRAM实时输出。
            rsp_data<={lfsr,~lfsr};
            if(pending) begin
                if(delay_left==0) begin rsp_valid<=1;rsp_data<=pending_data;pending<=0;responses=responses+1;end
                else delay_left<=delay_left-1;
            end
            if(rd_valid[1] && scratch_ready) begin
                if(pending) begin $display("[ERROR] multiple scratch requests");errors=errors+1;end
                pending<=1;delay_left<=lfsr[10:8];pending_data<=scratch_word(rd_addr[1]);requests=requests+1;
            end
        end
    end

    always @(posedge clk) begin
        if(!rst_n) begin sent=0;received=0;held_out=0;held_read=0;input_fire=0;requests=0;responses=0;end
        else begin
            // 与四状态参考核对控制时序；不再要求与原三状态版同周期。
            for(j=0;j<2;j=j+1) begin
                if({ready[j],busy[j],rd_valid[j],ov[j],error[j]}!==
                   {ready[j+2],busy[j+2],rd_valid[j+2],ov[j+2],error[j+2]} ||
                   rd_addr[j]!==rd_addr[j+2]) begin
                    $display("[ERROR] residual cycle mismatch mode=%0d cycle=%0d",j,cycles);errors=errors+1;
                end
                if(ov[j] && {oa[j],od[j],os[j]}!=={oa[j+2],od[j+2],os[j+2]}) begin
                    $display("[ERROR] residual legacy output mismatch mode=%0d",j);errors=errors+1;
                end
            end
            bundle0={oa[0],od[0],os[0]};bundle1={oa[1],od[1],os[1]};
            if(held_out && (!ov[0] || !ov[1] || bundle0!==held_bundle0 || bundle1!==held_bundle1)) errors=errors+1;
            held_out=ov[1] && !out_ready;held_bundle0=bundle0;held_bundle1=bundle1;
            if(held_out) output_stalls=output_stalls+1;
            if(held_read && (!rd_valid[1] || rd_addr[1]!==held_read_addr)) errors=errors+1;
            held_read=rd_valid[1] && !scratch_ready;held_read_addr=rd_addr[1];
            if(held_read) read_stalls=read_stalls+1;
            input_fire=in_valid && ready[1];
            if(in_valid && !ready[1]) input_stalls=input_stalls+1;
            if(input_fire) begin
                if(ov[1] && out_ready) overlap=overlap+1;
                if(addr<base || ((addr-base)%8)!=0) bad_addresses=bad_addresses+1;
                identity_word=scratch_word((addr-base)>>3);
                expected0[sent]=0;expected1[sent]=0;
                for(k=0;k<4;k=k+1) if(strb[2*k] || strb[2*k+1]) begin
                    expected0[sent][16*k+:16]=ref_sat16($signed(payload[16*k+:16])+$signed(identity_word[16*k+:16]));
                    ialigned=ref_sat16(ref_requant(identity_word[16*k+:16],params[31:0],params[39:32]));
                    maligned=ref_sat16(ref_requant(payload[16*k+:16],params[71:40],params[79:72]));
                    expected1[sent][16*k+:16]=ref_sat16($signed(ialigned)+ref_requant(maligned,params[111:80],params[119:112]));
                end
                expected_addr[sent]=addr;expected_strb[sent]=strb;strb_seen[strb]=1;
                sent=sent+1;
            end
            if(ov[1] && out_ready) begin
                if(received>=sent || {oa[0],od[0],os[0]}!=={expected_addr[received],expected0[received],expected_strb[received]} ||
                   {oa[1],od[1],os[1]}!=={expected_addr[received],expected1[received],expected_strb[received]}) begin
                    $display("[ERROR] residual scoreboard beat=%0d",received);errors=errors+1;
                end
                received=received+1;
            end
        end
    end

    // 上升沿消费响应后必须进入WAIT2且不能提前输出；再下一沿输出并回IDLE。
    // 在下降沿观察NBA完成后的状态，避免测试与DUT上升沿更新之间的竞态。
    always @(posedge clk) begin
        if(!rst_n) begin expect_wait2=0;expect_output=0;end
        else begin
            expect_wait2=(g_unit[1].g_new.dut.state==3'd2) && rsp_valid;
            expect_output=(g_unit[1].g_new.dut.state==3'd3);
            if(expect_output) wait2_checks=wait2_checks+1;
        end
    end
    always @(negedge clk) begin
        if(rst_n) begin
            if(expect_wait2 && (g_unit[1].g_new.dut.state!==3'd3 || ov!==4'b0000 ||
                ready!==4'b0000 || busy!==4'b1111 || rd_valid!==4'b0000)) begin
                $display("[ERROR] S_WAIT response must latch and enter S_WAIT2 without output");errors=errors+1;
            end
            if(expect_output && (g_unit[1].g_new.dut.state!==3'd0 || ov!==4'b1111)) begin
                $display("[ERROR] S_WAIT2 must output on next edge and return IDLE");errors=errors+1;
            end
        end
    end

    initial begin
        for(j=0;j<256;j=j+1) strb_seen[j]=0;
        // 完整shift域与INT16/INT32极值交叉；Guard不用Sticky，负半值必须远离零。
        for(s_idx=0;s_idx<256;s_idx=s_idx+1)
            for(v_idx=0;v_idx<7;v_idx=v_idx+1)
                for(m_idx=0;m_idx<7;m_idx=m_idx+1) begin
                    case(v_idx) 0:v=0;1:v=1;2:v=-1;3:v=32767;4:v=-32768;5:v=3;6:v=-3;endcase
                    case(m_idx) 0:multiplier=0;1:multiplier=1;2:multiplier=-1;
                        3:multiplier=32'sh7fffffff;4:multiplier=32'sh80000000;5:multiplier=65535;6:multiplier=-65535;endcase
                    check_arithmetic(v,multiplier,s_idx);
                end
        // 穷举全部INT16输入，覆盖零移位/半值/符号/最大乘积/47和48的分界。
        for(v_idx=-32768;v_idx<=32767;v_idx=v_idx+1) begin
            check_arithmetic(v_idx,1,0);check_arithmetic(v_idx,-1,1);
            check_arithmetic(v_idx,32'sh80000000,31);
            check_arithmetic(v_idx,32'sh80000000,47);
            check_arithmetic(v_idx,32'sh7fffffff,48);
        end
        for(m_idx=65530;m_idx<=65540;m_idx=m_idx+1) begin
            check_arithmetic(1,m_idx,0);check_arithmetic(-1,m_idx,0);
            check_arithmetic(1,m_idx*2+1,1);check_arithmetic(-1,m_idx*2+1,1);
        end
        for(j=0;j<40000;j=j+1) begin
            v=$random(seed);multiplier=$random(seed);shift=$random(seed);
            check_arithmetic(v,multiplier,shift);
        end
        $display("[RESIDUAL_ARITH] cases=%0d identity_sum_checks=%0d",arith_checks,sum_checks);

        repeat(5) @(negedge clk);rst_n=1;
        // 在等待SRAM响应期间复位，确保未完成事务不会泄漏到新一轮。
        make_input(0);in_valid=1;scratch_ready=1;
        repeat(2) @(negedge clk);rst_n=0;in_valid=0;
        repeat(3) @(negedge clk);rst_n=1;reset_flush=reset_flush+1;
        // 单独覆盖新增WAIT2寄存级中的复位，旧事务不得在复位后输出。
        make_input(1);in_valid=1;scratch_ready=1;out_ready=0;
        @(negedge clk);in_valid=0;
        while(g_unit[1].g_new.dut.state!=3'd3 && cycles<200) @(negedge clk);
        if(g_unit[1].g_new.dut.state!=3'd3) begin
            $display("[ERROR] reset test failed to reach S_WAIT2");errors=errors+1;
        end
        rst_n=0;
        repeat(3) @(negedge clk);
        if(ov!==0 || busy!==0) errors=errors+1;
        rst_n=1;reset_flush=reset_flush+1;
        while(received<1032 && cycles<100000) begin
            out_ready=(cycles%101>=55) && lfsr[3];
            scratch_ready=lfsr[1];
            if(!in_valid || input_fire) begin
                in_valid=(sent<1032) && (sent<64 || lfsr[0]);
                if(in_valid) make_input(sent);
            end
            lfsr={lfsr[30:0],lfsr[31]^lfsr[21]^lfsr[1]^lfsr[0]};
            @(negedge clk);
        end
        in_valid=0;out_ready=1;scratch_ready=1;
        repeat(5) @(negedge clk);
        for(j=0;j<256;j=j+1) if(!strb_seen[j]) errors=errors+1;
        if(sent!=1032 || received!=1032 || requests!=1032 || responses!=1032 || busy!==4'b0000 ||
           ov!==4'b0000 || error!==4'b1111 || input_stalls==0 || read_stalls==0 || output_stalls==0 ||
           overlap==0 || reset_flush!=2 || bad_addresses!=8 || wait2_checks!=1032) errors=errors+1;
        // error按原版为sticky，复位后应清除；不得借此轮优化偷偷改变错误协议。
        completed_beats=received;
        rst_n=0;repeat(3) @(negedge clk);
        if(error!==0 || busy!==0 || ov!==0) errors=errors+1;
        $display("[RESIDUAL_COVERAGE] beats=%0d modes=2 strobes=256 input_stall=%0d read_stall=%0d output_stall=%0d overlap=%0d bad_addr=%0d",completed_beats,input_stalls,read_stalls,output_stalls,overlap,bad_addresses);
        $display("[WAIT2_COVERAGE] output_cycles=%0d reset_flush=%0d",wait2_checks,reset_flush);
        if(errors==0) $display("[TB_PASS] residual48 Guard sat16 clip17 sum18: arithmetic + four-state pipeline cycles + stalls + strobes + reset/errors");
        else $display("[TB_FAIL] residual optimization errors=%0d",errors);
        $finish;
    end
    initial begin #2500000;$display("[TB_FAIL] residual timeout");$finish;end
endmodule
