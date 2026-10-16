`timescale 1ns/1ps
// 普通wire扫描输入的功能/DFT语义测试：0允许空闲停钟，1强制ICG透传。
// 不用force，不在RTL内部把scan硬编码成0；生产库绑定仍由服务器负责。
module tb_mesh_scan_port_compat;
    reg clk=0,rst_n=0,scan_enable=0;
    integer root_edges=0,mesh_edges=0,errors=0;
    integer before_edges,after_edges;
    reg pulse_seen=0;
    real rise_time;
    always #10 clk=~clk;
    conv1d_controller dut (
        .clk(clk),.rst_n(rst_n),.mesh_scan_test_en(scan_enable),.start(1'b0),
        .busy(),.done(),.error(),
        .cfg_input_base(32'd0),.cfg_output_base(32'd0),.cfg_weight_base(32'd0),
        .cfg_cin(9'd1),.cfg_cout(9'd8),.cfg_kernel(5'd1),.cfg_stride(4'd1),
        .cfg_dilation(4'd1),.cfg_left_pad(8'd0),.cfg_input_length(9'd1),
        .cfg_output_length(9'd1),.cfg_k_total(12'd1),.cfg_k_groups(10'd1),
        .cfg_n_groups(6'd1),.cfg_elu_enable(1'b0),.cfg_relu_enable(1'b0),
        .elu_lut_cfg_valid(1'b0),.elu_lut_cfg_ready(),.elu_lut_cfg_addr(7'd0),.elu_lut_cfg_wdata(8'd0),
        .act_rd_req_valid(),.act_rd_req_ready(1'b0),.act_rd_addr0(),.act_rd_addr1(),
        .act_rd_addr2(),.act_rd_addr3(),.act_rd_mask(),.act_rd_rsp_valid(1'b0),
        .act_rd_rsp_ready(),.act_rd_data(64'd0),.wgt_rd_req_valid(),.wgt_rd_req_ready(1'b0),
        .wgt_rd_addr(),.wgt_rd_rsp_valid(1'b0),.wgt_rd_rsp_ready(),.wgt_rd_data(256'd0),
        .param_rd_req_valid(),.param_rd_req_ready(1'b0),.param_rd_output_group(),
        .param_rd_rsp_valid(1'b0),.param_rd_rsp_ready(),.param_bias_data(512'd0),
        .param_mult_data(256'd0),.param_shift_data(64'd0),.param_zero_point_data(64'd0),
        .out_wr_valid(),.out_wr_ready(1'b0),.out_wr_addr(),.out_wr_data(),.out_wr_strb()
    );
    always @(posedge clk) if(rst_n) root_edges=root_edges+1;
    always @(posedge dut.mesh_gclk) if(rst_n) begin
        mesh_edges=mesh_edges+1;rise_time=$realtime;pulse_seen=1;
        if(clk!==1'b1) errors=errors+1;
    end
    always @(negedge dut.mesh_gclk) if(rst_n && pulse_seen) begin
        if($realtime-rise_time!=10.0) errors=errors+1;
        pulse_seen=0;
    end
    initial begin
        repeat(4) @(negedge clk);rst_n=1;
        repeat(8) @(negedge clk);
        if(mesh_edges!=0 || dut.mesh_gclk!==0) errors=errors+1;
        before_edges=mesh_edges;
        #1;scan_enable=1; // 在时钟低电平更新，等待锁存后完整透传
        repeat(7) @(negedge clk);
        after_edges=mesh_edges;
        if(after_edges-before_edges!=7) errors=errors+1;
        #1;scan_enable=0;
        repeat(8) @(negedge clk);
        if(mesh_edges!=after_edges || dut.mesh_gclk!==0) errors=errors+1;
        #1;scan_enable=1;
        repeat(5) @(negedge clk);
        if(mesh_edges-after_edges!=5) errors=errors+1;
        if(errors==0) $display("[TB_PASS] plain scan input: idle gate off, scan bypass 7+5 complete pulses, gate restored; root=%0d mesh=%0d",root_edges,mesh_edges);
        else $display("[TB_FAIL] plain scan input errors=%0d root=%0d mesh=%0d",errors,root_edges,mesh_edges);
        $finish;
    end
    initial begin #10000;$display("[TB_FAIL] scan input timeout");$finish;end
endmodule
