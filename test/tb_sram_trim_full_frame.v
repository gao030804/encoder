`timescale 1ns/1ps
// 在原Golden完整回归之外核对实际展开容量和每一笔SRAM地址，避免仅修改注释/参数。
module tb_sram_trim_full_frame #(
    parameter integer FRAME_COUNT=10,
    parameter integer CLOCK_HZ=50000000,
    parameter LOG_PREFIX="logs/sram44_50mhz/"
);
    tb_qor_p0_full_frame #(.FRAME_COUNT(FRAME_COUNT),.CLOCK_HZ(CLOCK_HZ),
        .LOG_PREFIX(LOG_PREFIX)) u_check();
`define TRIM_TEST u_check.u_test
`define TRIM_ENC u_check.u_test.u_dut.u_encoder
`define TRIM_GLOBAL u_check.u_test.u_dut.u_encoder.u_engine_flash.g_encoder_weight_hierarchy.u_weight_hierarchy
`define TRIM_ACT u_check.u_test.u_dut.u_encoder.u_activation_buffer
`define TRIM_RES u_check.u_test.u_dut.u_encoder.u_residual_scratch
    integer global_accesses=0,activation_accesses=0,residual_accesses=0;
    integer max_global=0,max_activation=0,max_residual=0;
    integer one_port_macros;
    reg reported=0;
    initial begin
        #1;
        if(`TRIM_GLOBAL.u_global_weight_sram.DEPTH!=7168 ||
           `TRIM_GLOBAL.u_global_weight_sram.BANKS!=14 ||
           `TRIM_GLOBAL.u_global_weight_sram.SLICES!=2 ||
           `TRIM_ACT.ROWS!=1536 || `TRIM_ACT.u_a_bank0.BANKS!=3 ||
           `TRIM_ACT.u_b_bank3.BANKS!=3 ||
           `TRIM_RES.DEPTH!=1536 || `TRIM_RES.u_residual_sram.BANKS!=3 ||
           `TRIM_ENC.u_capture.MAX_BYTES!=12288) begin
            $display("[TB_FAIL] SRAM trim configuration did not reach actual instantiated storage");$finish;
        end
        // 单口宏：Global28+Local4+Activation24+Residual3+RVQ6=65。
        // 首层128B是寄存器缓存，不计宏；8个累加器与2个输出环1W1R宏保持不变。
        one_port_macros=`TRIM_GLOBAL.u_global_weight_sram.BANKS*2 +
            `TRIM_GLOBAL.u_local_weight_sram_a.SLICES + `TRIM_GLOBAL.u_local_weight_sram_b.SLICES +
            8*`TRIM_ACT.u_a_bank0.BANKS + `TRIM_RES.u_residual_sram.BANKS +
            u_check.u_test.u_dut.u_backend.u_storage.BANKS*2;
        if(one_port_macros!=65) begin $display("[TB_FAIL] incorrect single-port macro count");$finish;end
        $display("[SRAM_TRIM_CONFIG] Global=7168x256 Activation=2x4x1536x16 Residual=1536x64 single_port_macros=%0d target_total_macros=75 saved_KiB=44",one_port_macros);
    end
    always @(posedge `TRIM_TEST.clk) if(`TRIM_TEST.rst_n) begin
        if(`TRIM_GLOBAL.u_global_weight_sram.req_i) begin
            if(`TRIM_GLOBAL.u_global_weight_sram.addr_i>=7168) begin
                $display("[TB_FAIL] Global address exceeds trimmed capacity");$finish;
            end
            global_accesses=global_accesses+1;
            if(`TRIM_GLOBAL.u_global_weight_sram.addr_i>max_global)
                max_global=`TRIM_GLOBAL.u_global_weight_sram.addr_i;
        end
        if(`TRIM_ACT.wr_fire) begin
            if(`TRIM_ACT.wr_row>=1536) begin $display("[TB_FAIL] Activation write OOB");$finish;end
            if(`TRIM_ACT.wr_row>max_activation) max_activation=`TRIM_ACT.wr_row;
            activation_accesses=activation_accesses+1;
        end
        if(`TRIM_ACT.rd_issue) begin
            if((`TRIM_ACT.decoded_need[0] && `TRIM_ACT.decoded_row[0]>=1536) ||
               (`TRIM_ACT.decoded_need[1] && `TRIM_ACT.decoded_row[1]>=1536) ||
               (`TRIM_ACT.decoded_need[2] && `TRIM_ACT.decoded_row[2]>=1536) ||
               (`TRIM_ACT.decoded_need[3] && `TRIM_ACT.decoded_row[3]>=1536)) begin
                $display("[TB_FAIL] Activation read OOB");$finish;
            end
            activation_accesses=activation_accesses+1;
        end
        if(`TRIM_RES.u_residual_sram.req_i) begin
            if(`TRIM_RES.u_residual_sram.addr_i>=1536) begin
                $display("[TB_FAIL] Residual address exceeds trimmed capacity");$finish;
            end
            if(`TRIM_RES.u_residual_sram.addr_i>max_residual)
                max_residual=`TRIM_RES.u_residual_sram.addr_i;
            residual_accesses=residual_accesses+1;
        end
        if(`TRIM_ACT.access_error || `TRIM_RES.error || `TRIM_GLOBAL.hierarchy_error) begin
            $display("[TB_FAIL] SRAM access error during full-frame regression");$finish;
        end
    end
    always @(negedge `TRIM_TEST.clk) begin
        if(!reported && ((FRAME_COUNT>1 && `TRIM_TEST.dma_frames==FRAME_COUNT) ||
            (FRAME_COUNT==1 && u_check.checks==1 && `TRIM_TEST.codec_level==0))) begin
            reported=1;
            if(max_global!=6911 || max_activation!=1279 || max_residual!=1279 ||
                global_accesses==0 || activation_accesses==0 || residual_accesses==0) begin
                $display("[TB_FAIL] SRAM access coverage global=%0d activation=%0d residual=%0d",max_global,max_activation,max_residual);$finish;
            end
            $display("[SRAM_TRIM_PASS] frames=%0d global_max=%0d activation_row_max=%0d residual_word_max=%0d accesses=%0d/%0d/%0d saved_bytes=45056",
                FRAME_COUNT,max_global,max_activation,max_residual,global_accesses,activation_accesses,residual_accesses);
        end
    end
`undef TRIM_TEST
`undef TRIM_ENC
`undef TRIM_GLOBAL
`undef TRIM_ACT
`undef TRIM_RES
endmodule
