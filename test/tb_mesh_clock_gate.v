`timescale 1ns/1ps
// 门控阵列与原始常开阵列逐拍对照；独立整数点积检查数据和标签。
module tb_mesh_clock_gate;
    reg clk = 0;
    reg rst_n = 0;
    reg phase_active = 0;
    reg test_en = 0;
    reg weight_load = 0, shadow_load = 0, weight_swap = 0;
    reg [255:0] weight_data = 0;
    reg act_valid = 0;
    reg [63:0] act_data = 0;
    reg [8:0] act_tag = 0;
    reg stall_mode = 0;
    integer cycles = 0, main_edges = 0, gate_edges = 0;
    integer errors = 0, head = 0, tail = 0, active_group = 0;
    integer stalls = 0, direct_loads = 0, shadow_loads = 0, swaps = 0;
    integer n, k, sum, a, w, group_index, sample_index;
    reg [207:0] expected [0:255];
    reg [8:0] expected_tag [0:255];
    wire gv, bv;
    wire [207:0] gd, bd;
    wire [8:0] gt, bt;
    // 连续多拍反压，且输入有气泡，避免只验证理想连续输入。
    wire sink_ready = !stall_mode || ((cycles % 23) < 13);
    wire ce = !gv || sink_ready;
    wire en = phase_active && ce;
    wire gclk;
    time last_rise;
    reg check_pulse = 0;

    always #10 clk = ~clk;
    always @(negedge clk) cycles = cycles + 1;
    mesh_icg u_icg (.clk_i(clk), .en_i(en), .test_en_i(test_en), .gclk_o(gclk));
    mesh_adapter u_gated (
        .clk(gclk), .rst_n(rst_n), .ce(ce),
        .weight_load(weight_load), .weight_shadow_load(shadow_load),
        .weight_swap(weight_swap), .weight_data(weight_data),
        .act_valid(act_valid), .act_data(act_data), .act_m_tag(act_tag),
        .psum_valid(gv), .psum_data(gd), .psum_m_tag(gt)
    );
    mesh_adapter u_baseline (
        .clk(clk), .rst_n(rst_n), .ce(ce),
        .weight_load(weight_load), .weight_shadow_load(shadow_load),
        .weight_swap(weight_swap), .weight_data(weight_data),
        .act_valid(act_valid), .act_data(act_data), .act_m_tag(act_tag),
        .psum_valid(bv), .psum_data(bd), .psum_m_tag(bt)
    );

    function integer weight_value;
        input integer g, row, col;
        begin weight_value = ((g*19 + row*7 + col*3) % 17) - 8; end
    endfunction
    function [255:0] make_weight;
        input integer g;
        integer row, col;
        begin
            make_weight = 0;
            for (row=0; row<4; row=row+1)
                for (col=0; col<8; col=col+1)
                    make_weight[8*(row*8+col)+:8] = weight_value(g,row,col);
        end
    endfunction

    always @(posedge gclk) begin
        if (rst_n) begin
            gate_edges = gate_edges + 1;
            if (clk !== 1'b1) begin $display("[ERROR] GCLK rose outside CLK high"); errors=errors+1; end
            last_rise = $time;
            check_pulse = 1;
        end
    end
    always @(negedge gclk) begin
        if (rst_n && check_pulse && $time-last_rise != 10) begin
            $display("[ERROR] truncated GCLK pulse width=%0t",$time-last_rise); errors=errors+1;
        end
        check_pulse = 0;
    end

    always @(posedge clk) begin
        if (rst_n) begin
            main_edges = main_edges + 1;
            if (phase_active) begin
                if (gv !== bv || (gv && (gd !== bd || gt !== bt))) begin
                    $display("[ERROR] gated/baseline mismatch cycle=%0d",cycles); errors=errors+1;
                end
                if (gv && !sink_ready) stalls = stalls + 1;
                if (ce && weight_load) direct_loads=direct_loads+1;
                if (ce && shadow_load) shadow_loads=shadow_loads+1;
                if (ce && weight_swap) swaps=swaps+1;
                if (ce && act_valid) begin
                    expected[tail]=0;
                    expected_tag[tail]=act_tag;
                    for (n=0; n<8; n=n+1) begin
                        sum=0;
                        for (k=0; k<4; k=k+1) begin
                            a=$signed(act_data[k*16+:16]);
                            w=weight_value(active_group,k,n);
                            sum=sum+a*w;
                        end
                        expected[tail][n*26+:26]=sum;
                    end
                    tail=tail+1;
                end
                if (gv && sink_ready) begin
                    if (head>=tail || gd!==expected[head] || gt!==expected_tag[head]) begin
                        $display("[ERROR] dot product/order item=%0d tag=%0d expected_tag=%0d actual=%h expected=%h",head,gt,expected_tag[head],gd,expected[head]);
                        errors=errors+1;
                    end
                    head=head+1;
                end
            end
        end
    end

    // 所有事务在下降沿建立，等 ce=1 的上升沿后才撤销，符合原始握手。
    task load_direct;
        input integer g;
        begin
            @(negedge clk); phase_active=1; weight_data=make_weight(g); weight_load=1;
            @(posedge clk); while (!ce) @(posedge clk);
            @(negedge clk); weight_load=0; active_group=g;
        end
    endtask
    task load_shadow;
        input integer g;
        begin
            weight_data=make_weight(g); shadow_load=1;
            @(posedge clk); while (!ce) @(posedge clk);
            @(negedge clk); shadow_load=0;
        end
    endtask
    task send_vector;
        input integer index;
        integer lane;
        begin
            act_tag=index; act_valid=1;
            for (lane=0; lane<4; lane=lane+1)
                act_data[lane*16+:16]=((index*997+lane*811)%60001)-30000;
            @(posedge clk); while (!ce) @(posedge clk);
            @(negedge clk); act_valid=0;
        end
    endtask

    initial begin
        repeat(4) @(negedge clk); rst_n=1;
        repeat(10) @(negedge clk);
        if(gate_edges!=0)begin $display("[ERROR] idle clock did not stop"); errors=errors+1; end
        stall_mode=1;
        for(group_index=0;group_index<3;group_index=group_index+1) begin
            if(group_index==0) load_direct(group_index);
            else begin
                weight_swap=1;
                @(posedge clk); while(!ce) @(posedge clk);
                @(negedge clk); weight_swap=0;active_group=group_index;
            end
            for(sample_index=0;sample_index<32;sample_index=sample_index+1)begin
                if(sample_index==8 && group_index<2) load_shadow(group_index+1);
                if(sample_index%7==3) repeat(3) @(negedge clk);
                send_vector(group_index*32+sample_index);
            end
            while(head<(group_index+1)*32) @(negedge clk);
            // WAIT_ACC 保守窗口，继续推进空 valid，模拟控制器的 V1 排空。
            repeat(20) @(negedge clk);
        end
        phase_active=0;stall_mode=0;
        k=gate_edges;repeat(20) @(negedge clk);
        if(gate_edges!=k)begin $display("[ERROR] post/idle GCLK still toggles");errors=errors+1;end

        // 高电平改变测试使能不能生成短脉冲；下一完整时钟才开始旁路。
        @(posedge clk);#3;test_en=1;
        #1;if(gclk!==0)begin $display("[ERROR] test enable caused mid-high edge");errors=errors+1;end
        repeat(3) @(negedge clk);
        @(posedge clk);#3;test_en=0;
        #1;if(gclk!==1)begin $display("[ERROR] test disable truncated high pulse");errors=errors+1;end
        repeat(3) @(negedge clk);
        // 停钟时仍须响应异步复位，不要求门控时钟出现上升沿。
        rst_n=0;#2;
        if(u_gated.u_mesh_core.weight_q!==0 || gv!==0)begin
            $display("[ERROR] asynchronous reset while clock stopped");errors=errors+1;
        end
        if(head!=96 || tail!=96 || stalls==0 || direct_loads!=1 || shadow_loads!=2 || swaps!=2 || gate_edges>=main_edges)begin
            $display("[ERROR] coverage incomplete");errors=errors+1;
        end
        $display("[MESH_ICG_STATS] main=%0d gated=%0d stalled=%0d vectors=%0d direct=%0d shadow=%0d swap=%0d",main_edges,gate_edges,stalls,head,direct_loads,shadow_loads,swaps);
        if(errors==0)$display("[TB_PASS] Mesh ICG baseline/dot/tag/load/shadow/swap/backpressure/drain/reset/test-enable");
        else $display("[TB_FAIL] errors=%0d",errors);
        $finish;
    end
    initial begin #1000000;$display("[TB_FAIL] Mesh ICG timeout");$finish;end
endmodule
