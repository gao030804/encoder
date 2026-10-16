`timescale 1ns/1ps
// 服务器兼容封装单测：通过 mesh_icg 的真实 TECH 分支运行，非直接替代模型。
// 正常比较条件：功能使能/扫描使能在高电平改变，在下一下降沿前稳定。
module tb_mesh_icg_tech;
    reg clk=0, en=0, scan=0, checking=0;
    reg ref_enable;
    wire gated_clk;
    wire reference_clk=clk & ref_enable;
    integer errors=0, pulses=0, before_pulses;
    real rise_time;
    reg pulse_active=0;
    always #10 clk=~clk;

    mesh_icg dut(.clk_i(clk),.en_i(en),.test_en_i(scan),.gclk_o(gated_clk));
    always @(*) if(!clk) ref_enable=en | scan;

    // 等非阻塞赋值稳定后比较。未复位 FF 在首个下降沿之前不保证已知。
    always @(clk) begin
        #1;
        if(checking && gated_clk!==reference_clk) errors=errors+1;
    end
    always @(posedge gated_clk) if(checking) begin
        pulses=pulses+1;
        rise_time=$realtime;
        pulse_active=1;
        if(clk!==1'b1) errors=errors+1;
    end
    always @(negedge gated_clk) if(checking && pulse_active) begin
        if($realtime-rise_time!=10.0) errors=errors+1;
        pulse_active=0;
    end

    // 高电平更新使能，当前脉冲不被截断，下一个完整脉冲按新使能产生。
    task run_window;
        input functional_en, scan_en;
        input integer cycles;
        integer expected;
        begin
            @(posedge clk); #2;
            en=functional_en; scan=scan_en;
            @(negedge clk); #2;
            before_pulses=pulses;
            repeat(cycles) @(negedge clk);
            #2;
            expected=(functional_en | scan_en) ? cycles : 0;
            if(pulses-before_pulses!=expected) errors=errors+1;
        end
    endtask

    initial begin
        repeat(3) @(negedge clk);
        #2; checking=1;
        run_window(0,0,5);
        run_window(1,0,7);
        run_window(0,0,5);
        run_window(0,1,5);
        run_window(1,1,4);
        run_window(0,0,5);

        // 明确测试与 latch 的边界差异：下降沿之后才打开使能，FF 不会立即跟随。
        // 此段不要求两模型相等，只检查 FF 等到下一个下降沿才采样。
        checking=0;
        @(negedge clk); #2; en=1;
        @(posedge clk); #2;
        if(gated_clk!==0 || reference_clk!==1) errors=errors+1;
        @(negedge clk);
        @(posedge clk); #2;
        if(gated_clk!==1) errors=errors+1;

        if(errors==0)
            $display("[TB_PASS] TECH negedge gate: functional/scan/stop/full-width pulses and late-low sampling boundary verified; pulses=%0d",pulses);
        else $display("[TB_FAIL] TECH gate errors=%0d",errors);
        $finish;
    end
    initial begin #20000; $display("[TB_FAIL] TECH gate timeout"); $finish; end
endmodule
