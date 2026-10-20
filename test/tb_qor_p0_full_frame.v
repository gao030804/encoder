`timescale 1ns/1ps
// 无UART ASIC核心：沿用原Flash启动、真实PCM与逐层/逐级Golden检查。
// 本封装只增加截止时间、投影新增状态覆盖和后处理Data/Tag停顿检查。
module tb_qor_p0_full_frame #(
    parameter integer FRAME_COUNT=10,
    parameter integer CLOCK_HZ=50000000,
    parameter LOG_PREFIX="logs/qor_p0_50mhz/"
);
    localparam real PERIOD_NS=1000000000.0/CLOCK_HZ;
    tb_flash_encoder_rvq_full_frame #(
        .FRAME_COUNT(FRAME_COUNT),.CLOCK_HZ(CLOCK_HZ),.LOG_PREFIX(LOG_PREFIX)
    ) u_test();
`define QOR_DENSE u_test.u_dut.u_encoder.u_engine_flash.u_engine.u_dense
    integer checks=0,worst_cycles=0,cycles;
    integer bias_count=0,mul_count=0,skid_stalls=0,skid_replaces=0;
    integer update_mul_count=0,update_rnd_count=0;
    reg held=0,reported=0;
    reg [319:0] held_data;
    reg [8:0] held_tag;
    real duration_ms;
    always @(posedge u_test.clk) begin
        if(!u_test.rst_n) held=0;
        else begin
            if(held && (!`QOR_DENSE.post_skid_valid ||
                `QOR_DENSE.post_skid_data!==held_data || `QOR_DENSE.post_skid_tag!==held_tag)) begin
                $display("[TB_FAIL] full-frame elastic response changed under backpressure");$finish;
            end
            held=`QOR_DENSE.post_skid_valid && !`QOR_DENSE.post_in_ready;
            held_data=`QOR_DENSE.post_skid_data;held_tag=`QOR_DENSE.post_skid_tag;
            if(held) skid_stalls=skid_stalls+1;
            if(`QOR_DENSE.post_skid_valid && `QOR_DENSE.post_in_ready &&
               `QOR_DENSE.acc_rd_rsp_valid && `QOR_DENSE.acc_rd_rsp_ready) skid_replaces=skid_replaces+1;
            if(u_test.u_dut.u_backend.state==23) bias_count=bias_count+1;
            if(u_test.u_dut.u_backend.state==24) mul_count=mul_count+1;
            if(u_test.u_dut.u_backend.state==19) update_mul_count=update_mul_count+1;
            if(u_test.u_dut.u_backend.state==25) update_rnd_count=update_rnd_count+1;
            if(u_test.codec_enqueued) begin
                cycles=u_test.cycle_count-u_test.frame_start_cycle;
                duration_ms=cycles*PERIOD_NS/1000000.0;
                if(cycles>worst_cycles) worst_cycles=cycles;
                checks=checks+1;
                $display("[QOR_DEADLINE] frame=%0d clock_hz=%0d cycles=%0d time_ms=%0.6f remaining_ms=%0.6f",u_test.frame_number,CLOCK_HZ,cycles,duration_ms,20.0-duration_ms);
                if(duration_ms>=20.0) begin $display("[TB_FAIL] frame deadline exceeded");$finish;end
            end
        end
    end
    always @(negedge u_test.clk) begin
        if(!reported && ((FRAME_COUNT>1 && u_test.dma_frames==FRAME_COUNT) ||
            (FRAME_COUNT==1 && checks==1 && u_test.codec_level==0))) begin
            reported=1;
            if(checks!=FRAME_COUNT || bias_count!=32*FRAME_COUNT || mul_count!=32*FRAME_COUNT ||
               update_mul_count!=288*FRAME_COUNT || update_rnd_count!=288*FRAME_COUNT ||
               skid_stalls==0 || skid_replaces==0) begin
                $display("[TB_FAIL] P0 coverage incomplete frames=%0d bias=%0d mul=%0d stalls=%0d replace=%0d",checks,bias_count,mul_count,skid_stalls,skid_replaces);$finish;
            end
            $display("[QOR_P0_PASS] frames=%0d clock_hz=%0d worst_cycles=%0d worst_ms=%0.6f bias=%0d mul=%0d skid_stall=%0d skid_replace=%0d",checks,CLOCK_HZ,worst_cycles,worst_cycles*PERIOD_NS/1000000.0,bias_count,mul_count,skid_stalls,skid_replaces);
            $display("[RVQ_UPDATE_PIPE_PASS] mul=%0d rnd=%0d",update_mul_count,update_rnd_count);
        end
    end
`undef QOR_DENSE
endmodule
