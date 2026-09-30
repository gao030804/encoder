// 本文件包含在整帧Testbench模块内。DUT内部信号仅用于监测及Boot总线连接，
// 不force、不直接写DUT存储，不将Golden反馈到计算数据路径。
reg aux_mode = 1;
reg aux_valid = 0;
reg aux_ready = 0;
reg [31:0] aux_addr = 0;
wire store_ready, store_valid;
wire [895:0] store_data;
reg [31:0] parameter_offsets [0:62]; // 地址配置ROM，只有偏移，不含量化数值
reg [255:0] flash_expected [0:6911]; // 仅比较用
reg [895:0] aux_result;
integer boot_writes = 0;
integer parameter_writes = 0;
integer cs_count = 0;
integer boot_command_cycle = -1;
integer local_a_count [0:62];
integer local_b_count [0:62];
integer act_reads [0:62];
integer parameter_reads [0:62];
integer g2l_active_cycles [0:62];
integer act_stall_cycles [0:62];
// 下列区间按同一个cycle_count计时。活动/等待计数可能互相重叠，不能相加作总延迟。
integer pcm_done_cycle = -1;
integer engine_sum_cycles;
integer inter_layer_cycles;
integer frame_total_cycles;
integer summary_file;
integer weight_stall_cycles [0:62];
integer mesh_load_cycles [0:62];
integer first_mesh_load [0:62];
integer last_mesh_load [0:62];
integer support_index;
integer report_file;
integer g2l_cycles;
integer aq_head = 0, aq_tail = 0, wq_head = 0, wq_tail = 0;
reg [15:0] activation_shadow [0:32767];
reg [63:0] act_expected_queue [0:1023];
reg [3:0] act_mask_queue [0:1023];
reg [31:0] weight_address_queue [0:1023];
integer monitor_lane;
reg latent_req = 0, latent_rsp_ready = 0;
reg [31:0] latent_addr = 0;
wire latent_ready, latent_valid;
wire [31:0] latent_base;
wire [63:0] latent_data;
reg [31:0] parameter_address_q;
integer parameter_byte;

flash_boot_parameter_store u_parameter_store (
    .clk(clk), .rst_n(rst_n),
    // 物理连线接收真实Boot握手，不从文件初始化Parameter SRAM。
    .boot_write(`HIER.global_dma_write),
    .boot_block(`HIER.global_addr), .boot_data(`HIER.dma_block_data),
    .boot_done(u_dut.u_encoder.first_weight_ready),
    .req_valid(aux_mode ? aux_valid : param_rd_req_valid),
    .req_ready(store_ready),
    .req_addr(aux_mode ? aux_addr : parameter_offsets[param_rd_layer]+112*param_rd_output_group),
    .rsp_valid(store_valid),
    .rsp_ready(aux_mode ? aux_ready : param_rd_rsp_ready), .rsp_data(store_data)
);
assign param_rd_req_ready = !aux_mode && store_ready;
assign param_rd_rsp_valid = !aux_mode && store_valid;
assign param_bias_data = store_data[511:0];
assign param_mult_data = store_data[767:512];
assign param_shift_data = store_data[831:768];
assign param_zero_point_data = store_data[895:832];

initial begin
    $readmemh("tb/data/flash_encoder_frame/expected_blocks.hex", flash_expected);
    $readmemh("tb/data/flash_encoder_frame/layer_parameter_offsets.hex", parameter_offsets);
    for (support_index=0; support_index<63; support_index=support_index+1) begin
        local_a_count[support_index]=0; local_b_count[support_index]=0;
        act_reads[support_index]=0; parameter_reads[support_index]=0;
        g2l_active_cycles[support_index]=0; act_stall_cycles[support_index]=0;
        weight_stall_cycles[support_index]=0; mesh_load_cycles[support_index]=0;
        first_mesh_load[support_index]=-1; last_mesh_load[support_index]=-1;
    end
end
always @(negedge flash_csn) cs_count = cs_count+1;

// 控制参数读取也使用同步SRAM握手。响应停留两拍，覆盖缓存反压。
task read_aux_record;
    input [31:0] address;
    begin
        @(negedge clk); aux_addr=address; aux_valid=1;
        @(posedge clk); while (!store_ready) @(posedge clk);
        @(negedge clk); aux_valid=0;
        while (!store_valid) @(negedge clk);
        aux_result=store_data;
        repeat (2) begin
            @(negedge clk);
            if (!store_valid || store_data !== aux_result) begin
                $display("[ERROR] parameter response changed under stall"); errors=errors+1;
            end
        end
        aux_ready=1;
        @(posedge clk); @(negedge clk); aux_ready=0;
    end
endtask

task load_boot_parameters;
    integer record_index;
    begin
        wait(u_dut.u_encoder.first_weight_ready);
        @(negedge clk);
        $display("[BOOT] Global writes=%0d Parameter writes=%0d CS=%0d cycle=%0d",
                  boot_writes,parameter_writes,cs_count,cycle_count);
        if (boot_writes != 6912 || parameter_writes != 1792 || cs_count != 3) begin
            $display("[ERROR] boot count mismatch"); errors=errors+1;
        end
        for (record_index=0; record_index<64; record_index=record_index+1) begin
            read_aux_record(32'h35000+16*record_index);
            residual_params_mem[record_index]=aux_result[119:0];
        end
        read_aux_record(32'h35400);
        pcm_multiplier_mem[0]=aux_result[31:0];
        pcm_shift_mem[0]=aux_result[39:32];
        aux_mode=0;
        $display("[BOOT] PCM and residual registers loaded via Parameter SRAM");
    end
endtask

always @(posedge clk) if (rst_n) begin
    if (pcm_frontend_done && pcm_done_cycle<0) pcm_done_cycle=cycle_count;
    if (`HIER.req_valid && !`HIER.req_ready)
        weight_stall_cycles[physical_layer]=weight_stall_cycles[physical_layer]+1;
    // Dense/Pointwise装载Active或Shadow权重寄存器；Depthwise使用专用控制器，计数为0。
    if (u_dut.u_encoder.u_engine_flash.u_engine.u_dense.mesh_direct_weight_load ||
        u_dut.u_encoder.u_engine_flash.u_engine.u_dense.mesh_shadow_weight_load) begin
        mesh_load_cycles[physical_layer]=mesh_load_cycles[physical_layer]+1;
        if (first_mesh_load[physical_layer]<0) first_mesh_load[physical_layer]=cycle_count;
        last_mesh_load[physical_layer]=cycle_count;
    end
    if (u_parameter_store.req_valid && store_ready) begin
        parameter_address_q=u_parameter_store.req_addr;
        if (parameter_address_q < 32'h28000 || parameter_address_q+112 > 32'h36000) begin
            $display("[TB_FAIL] parameter address outside image"); $finish;
        end
    end
    if (store_valid && u_parameter_store.rsp_ready)
        for (parameter_byte=0;parameter_byte<112;parameter_byte=parameter_byte+1)
            if (store_data[8*parameter_byte +: 8] !==
                flash_expected[(parameter_address_q+parameter_byte)>>5][8*((parameter_address_q+parameter_byte)%32) +: 8]) begin
                $display("[ERROR] Parameter SRAM byte addr=%h",parameter_address_q+parameter_byte); errors=errors+1;
            end
    if (`HIER.dma_cmd_valid && `HIER.dma_cmd_ready) boot_command_cycle=cycle_count;
    if (`HIER.global_dma_write) begin
        if (`HIER.global_addr !== boot_writes[12:0] ||
            `HIER.dma_block_data !== flash_expected[boot_writes]) begin
            $display("[ERROR] Flash/Global mismatch block=%0d",boot_writes); errors=errors+1;
        end
        boot_writes=boot_writes+1;
    end
    if (u_parameter_store.write_en) parameter_writes=parameter_writes+1;
    if (`HIER.bank_a_sram_write) local_a_count[physical_layer]=local_a_count[physical_layer]+1;
    if (`HIER.control_state==5 || `HIER.control_state==6)
        g2l_active_cycles[physical_layer]=g2l_active_cycles[physical_layer]+1;
    if (u_dut.u_encoder.eng_act_req_valid && !u_dut.u_encoder.eng_act_req_ready)
        act_stall_cycles[physical_layer]=act_stall_cycles[physical_layer]+1;
    if (`HIER.bank_b_sram_write) local_b_count[physical_layer]=local_b_count[physical_layer]+1;
    if (param_rd_req_valid && param_rd_req_ready) parameter_reads[physical_layer]=parameter_reads[physical_layer]+1;

    // 实际写入作为Activation SRAM影子；Golden逐层对拍在主文件独立进行。
    if (input_wr_valid && input_wr_ready)
        for (monitor_lane=0;monitor_lane<4;monitor_lane=monitor_lane+1)
            if (input_wr_strb[2*monitor_lane +: 2] == 2'b11)
                activation_shadow[(input_wr_addr>>1)+monitor_lane]=input_wr_data[16*monitor_lane +: 16];
    if (layer_write_fire)
        for (monitor_lane=0;monitor_lane<4;monitor_lane=monitor_lane+1)
            if (layer_write_strb[2*monitor_lane +: 2] == 2'b11)
                activation_shadow[(layer_write_addr>>1)+monitor_lane]=layer_write_data[16*monitor_lane +: 16];
    if (u_dut.u_encoder.eng_act_req_valid && u_dut.u_encoder.eng_act_req_ready) begin
        act_reads[physical_layer]=act_reads[physical_layer]+1;
        act_mask_queue[aq_tail%1024]=u_dut.u_encoder.eng_act_mask;
        act_expected_queue[aq_tail%1024]={
            activation_shadow[u_dut.u_encoder.eng_act_addr3>>1],activation_shadow[u_dut.u_encoder.eng_act_addr2>>1],
            activation_shadow[u_dut.u_encoder.eng_act_addr1>>1],activation_shadow[u_dut.u_encoder.eng_act_addr0>>1]};
        aq_tail=aq_tail+1;
        if (aq_tail-aq_head>1024) begin $display("[TB_FAIL] activation queue overflow"); $finish; end
    end
    if (u_dut.u_encoder.eng_act_rsp_valid && u_dut.u_encoder.eng_act_rsp_ready) begin
        if (aq_head>=aq_tail) begin $display("[TB_FAIL] unexpected activation response"); $finish; end
        for (monitor_lane=0;monitor_lane<4;monitor_lane=monitor_lane+1)
            if (act_mask_queue[aq_head%1024][monitor_lane] &&
                u_dut.u_encoder.eng_act_data[16*monitor_lane +: 16] !== act_expected_queue[aq_head%1024][16*monitor_lane +: 16]) begin
                $display("[ERROR] Activation SRAM payload layer=%0d lane=%0d",physical_layer,monitor_lane); errors=errors+1;
            end
        aq_head=aq_head+1;
    end
    if (`HIER.req_valid && `HIER.req_ready) begin
        weight_address_queue[wq_tail%1024]=`HIER.req_addr; wq_tail=wq_tail+1;
        if (wq_tail-wq_head>1024) begin $display("[TB_FAIL] weight queue overflow"); $finish; end
    end
    if (`HIER.rsp_valid && `HIER.rsp_ready) begin
        if (wq_head>=wq_tail) begin $display("[TB_FAIL] unexpected weight response"); $finish; end
        if (`HIER.rsp_data !== flash_expected[weight_address_queue[wq_head%1024]>>5]) begin
            $display("[ERROR] Local weight payload addr=%h",weight_address_queue[wq_head%1024]); errors=errors+1;
        end
        wq_head=wq_head+1;
    end
end

// 完成标志之后经顶层结果读接口读回64个INT16 latent，验证最终SRAM留存值。
task check_final_latent;
    integer word_index, final_lane, final_errors;
    begin
        final_errors=errors;
        for (word_index=0;word_index<16;word_index=word_index+1) begin
            @(negedge clk); latent_addr=latent_base+8*word_index; latent_req=1;
            @(posedge clk); while (!latent_ready) @(posedge clk);
            @(negedge clk); latent_req=0;
            while (!latent_valid) @(negedge clk);
            for (final_lane=0;final_lane<4;final_lane=final_lane+1)
                if (latent_data[16*final_lane +: 16] !== golden_layers[62*8192+4*word_index+final_lane]) begin
                    $display("[ERROR] final latent index=%0d",4*word_index+final_lane); errors=errors+1;
                end
            latent_rsp_ready=1;
            @(posedge clk); @(negedge clk); latent_rsp_ready=0;
        end
        if (errors==final_errors) $display("[LATENT_PASS] all 64 INT16 final SRAM values match Golden");
    end
endtask

task print_flash_frame_report;
    integer report_layer;
    reg [1023:0] timing_path, summary_path;
    begin
        if(FRAME_COUNT>1) begin
            $sformat(timing_path,"logs/multiframe_%0d%0d%0d_layers.csv",
                     frame_number/100,(frame_number/10)%10,frame_number%10);
            $sformat(summary_path,"logs/multiframe_%0d%0d%0d_summary.csv",
                     frame_number/100,(frame_number/10)%10,frame_number%10);
        end else begin
            timing_path="logs/flash_encoder_rvq_timing.csv";
            summary_path="logs/flash_encoder_rvq_summary.csv";
        end
        report_file=$fopen(timing_path,"w");
        if (!report_file) begin $display("[TB_FAIL] cannot open timing report"); $finish; end
        $fwrite(report_file,"layer,engine_cycles,g2l_prefetch_cycles,weight_reads,activation_requests,parameter_requests,local_a_writes_during_layer,local_b_writes_during_layer,output_bytes,g2l_active_cycles,activation_request_stall_cycles,weight_request_stall_cycles,dense_mesh_load_cycles,first_mesh_load_cycle,last_mesh_load_cycle\n");
        engine_sum_cycles=0;
        for (report_layer=0;report_layer<63;report_layer=report_layer+1) begin
            g2l_cycles=(report_layer==0)?0:prefetch_done_cycle[report_layer]-prefetch_start_cycle[report_layer];
            engine_sum_cycles=engine_sum_cycles+compute_done_cycle[report_layer]-compute_start_cycle[report_layer];
            $display("[LAYER_TIMING] layer=%0d engine_ns=%0d g2l_prepare_ns=%0d weight_stall_cycles=%0d act_stall_cycles=%0d mesh_load_cycles=%0d",report_layer,
                (compute_done_cycle[report_layer]-compute_start_cycle[report_layer])*CLOCK_PERIOD_NS,
                g2l_cycles*CLOCK_PERIOD_NS,
                weight_stall_cycles[report_layer],act_stall_cycles[report_layer],mesh_load_cycles[report_layer]);
            $fwrite(report_file,"%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",report_layer,
                compute_done_cycle[report_layer]-compute_start_cycle[report_layer],g2l_cycles,
                weight_sram_read_count[report_layer],act_reads[report_layer],parameter_reads[report_layer],
                local_a_count[report_layer],local_b_count[report_layer],actual_output_bytes[report_layer],
                g2l_active_cycles[report_layer],act_stall_cycles[report_layer],weight_stall_cycles[report_layer],
                mesh_load_cycles[report_layer],first_mesh_load[report_layer],last_mesh_load[report_layer]);
        end
        $fclose(report_file);
        // 无重叠分解：输入准备 + 63层引擎区间 + 层间调度等开销 = 帧端到端时间。
        frame_total_cycles=encoder_done_cycle-frame_start_cycle;
        inter_layer_cycles=encoder_done_cycle-encoder_start_cycle-engine_sum_cycles;
        if (pcm_done_cycle<frame_start_cycle || pcm_done_cycle>encoder_start_cycle || inter_layer_cycles<0) begin
            $display("[ERROR] invalid timing boundaries"); errors=errors+1;
        end
        $display("[FRAME_TIMING] pcm_to_latent cycles=%0d ns=%0d ms=%0.6f (includes all in-frame transfers)",
                 frame_total_cycles,frame_total_cycles*CLOCK_PERIOD_NS,
                 frame_total_cycles*CLOCK_PERIOD_NS/1000000.0);
        $display("[FRAME_BREAKDOWN] pcm_input_ns=%0d start_setup_ns=%0d engine_sum_ns=%0d inter_layer_ns=%0d",
            (pcm_done_cycle-frame_start_cycle)*CLOCK_PERIOD_NS,
            (encoder_start_cycle-pcm_done_cycle)*CLOCK_PERIOD_NS,
            engine_sum_cycles*CLOCK_PERIOD_NS,inter_layer_cycles*CLOCK_PERIOD_NS);
        summary_file=$fopen(summary_path,"w");
        if (!summary_file) begin $display("[TB_FAIL] cannot open summary"); $finish; end
        $fwrite(summary_file,"metric,cycles,time_ns,time_ms\n");
        $fwrite(summary_file,"pcm_to_latent,%0d,%0d,%0.6f\n",frame_total_cycles,
                frame_total_cycles*CLOCK_PERIOD_NS,frame_total_cycles*CLOCK_PERIOD_NS/1000000.0);
        $fwrite(summary_file,"pcm_input,%0d,%0d,%0.6f\n",pcm_done_cycle-frame_start_cycle,
                (pcm_done_cycle-frame_start_cycle)*CLOCK_PERIOD_NS,
                (pcm_done_cycle-frame_start_cycle)*CLOCK_PERIOD_NS/1000000.0);
        $fwrite(summary_file,"input_done_to_encoder_start,%0d,%0d,%0.6f\n",encoder_start_cycle-pcm_done_cycle,
                (encoder_start_cycle-pcm_done_cycle)*CLOCK_PERIOD_NS,
                (encoder_start_cycle-pcm_done_cycle)*CLOCK_PERIOD_NS/1000000.0);
        $fwrite(summary_file,"engine_sum_including_stalls,%0d,%0d,%0.6f\n",engine_sum_cycles,
                engine_sum_cycles*CLOCK_PERIOD_NS,engine_sum_cycles*CLOCK_PERIOD_NS/1000000.0);
        $fwrite(summary_file,"inter_layer_and_control,%0d,%0d,%0.6f\n",inter_layer_cycles,
                inter_layer_cycles*CLOCK_PERIOD_NS,inter_layer_cycles*CLOCK_PERIOD_NS/1000000.0);
        $fwrite(summary_file,"reset_to_latent,%0d,%0d,%0.6f\n",encoder_done_cycle,
                encoder_done_cycle*CLOCK_PERIOD_NS,encoder_done_cycle*CLOCK_PERIOD_NS/1000000.0);
        $fwrite(summary_file,"reset_to_pcm_start,%0d,%0d,%0.6f\n",frame_start_cycle,
                frame_start_cycle*CLOCK_PERIOD_NS,frame_start_cycle*CLOCK_PERIOD_NS/1000000.0);
        $fclose(summary_file);
        if (cs_count!=3 || aq_head!=aq_tail || wq_head!=wq_tail) begin
            $display("[ERROR] unexpected Flash reuse or outstanding reads"); errors=errors+1;
        end
        $display("[TIMING] boot_command_to_resident_ready_ns=%0d",
                 (boot_weight_done_cycle-boot_command_cycle)*CLOCK_PERIOD_NS);
        $display("[TIMING] PCM_start_to_latent_ns=%0d",
                 (encoder_done_cycle-frame_start_cycle)*CLOCK_PERIOD_NS);
        $display("[TIMING] encoder_start_to_latent_ns=%0d",
                 (encoder_done_cycle-encoder_start_cycle)*CLOCK_PERIOD_NS);
        $display("[TIMING] reset_release_to_latent_ns=%0d",encoder_done_cycle*CLOCK_PERIOD_NS);
        $display("[COUNTS] act_requests=%0d weight_requests=%0d Flash_CS=%0d",aq_tail,wq_tail,cs_count);
    end
endtask
initial begin
    #(100000000.0*FRAME_COUNT);
    $display("[TB_FAIL] full-frame watchdog cycle=%0d layer=%0d",cycle_count,physical_layer);
    $finish;
end
