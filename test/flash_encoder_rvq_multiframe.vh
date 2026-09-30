// 多帧验证控制，不复位DUT、不写内部SRAM。Golden仅供观察比较。
// 先积压3帧，再并行DMA出队，证明编码完成不依赖消费者立即接收。
reg [63:0] dma_expected [0:9];
reg [31:0] dma_timestamps [0:9];
integer dma_enqueue_cycles [0:9];
integer dma_input_cycles [0:9];
integer enqueued_frames=0, dma_frames=0, max_queue_level=0;
integer dma_stalled_cycles=0, dma_file=0, dma_timing_file=0;
reg dma_was_stalled=0;
reg [127:0] dma_held_record;
reg [1023:0] frame_dir, frame_path;
reg [7:0] input_mode_mem [0:0];
reg [255:0] input_sha256_mem [0:0];

// Verilog 的 %03d 在 ModelSim 10.6c 中以空格补宽度，不能用于 frame_000 文件名。
// 文件不存在时立即停止，避免 $readmemh 警告后把 X 当成计算错误继续跑完整帧。
task require_frame_file;
    input [1023:0] path;
    integer handle;
    begin
        handle=$fopen(path,"r");
        if(handle==0) begin
            $display("[TB_FAIL] missing frame Golden: %0s",path);
            $finish;
        end
        $fclose(handle);
    end
endtask

initial if (FRAME_COUNT>1) begin
    if(FRAME_COUNT!=10) begin $display("[TB_FAIL] only FRAME_COUNT=1 or 10 supported");$finish;end
    require_frame_file("tb/data/encoder_rvq_multiframe/input_mode.hex");
    require_frame_file("tb/data/encoder_rvq_multiframe/input_sha256.hex");
    $readmemh("tb/data/encoder_rvq_multiframe/input_mode.hex",input_mode_mem);
    $readmemh("tb/data/encoder_rvq_multiframe/input_sha256.hex",input_sha256_mem);
    case (input_mode_mem[0])
        8'd1: $display("[INPUT_SOURCE] mode=raw_pcm_independent_frames sha256=%064h",input_sha256_mem[0]);
        8'd2: $display("[INPUT_SOURCE] mode=wav_independent_frames sha256=%064h",input_sha256_mem[0]);
        8'd0: $display("[INPUT_SOURCE] mode=derived_single_frame_test_vectors sha256=%064h",input_sha256_mem[0]);
        default: begin $display("[TB_FAIL] unknown input source mode");$finish;end
    endcase
    $readmemh("tb/data/encoder_rvq_multiframe/packed_all.hex",dma_expected);
    dma_file=$fopen("logs/encoded_records_multiframe.hex","w");
    dma_timing_file=$fopen("logs/encoder_rvq_multiframe_dma.csv","w");
    if(!dma_file || !dma_timing_file) begin $display("[TB_FAIL] cannot open DMA logs");$finish;end
    $fdisplay(dma_timing_file,"frame,pcm_to_enqueue_ns,queue_wait_ns,pcm_to_dma_ns,timestamp_us");
end

// 与编码驱动独立。延迟消费后按确定性规则插入反压，便于重现。
always @(negedge clk) if(FRAME_COUNT>1) begin
    codec_ready = rst_n && enqueued_frames>=3 && (cycle_count%97>=35);
end
always @(posedge clk) if(rst_n && FRAME_COUNT>1) begin
    if(encoder_start && encoder_start_ready) dma_timestamps[frame_number]=cycle_count/CYCLES_PER_US;
    if(codec_level>max_queue_level) max_queue_level=codec_level;
    if(dma_was_stalled && (!codec_valid || codec_record!==dma_held_record)) begin
        $display("[ERROR] DMA changed under backpressure");errors=errors+1;
    end
    dma_was_stalled=codec_valid && !codec_ready;
    dma_held_record=codec_record;
    if(dma_was_stalled) dma_stalled_cycles=dma_stalled_cycles+1;
    if(codec_enqueued) begin
        if(enqueued_frames!=frame_number || u_dut.backend_indices!==dma_expected[frame_number]) begin
            $display("[ERROR] enqueue frame/order/indices frame=%0d",frame_number);errors=errors+1;
        end
        dma_enqueue_cycles[frame_number]=cycle_count;
        dma_input_cycles[frame_number]=frame_start_cycle;
        enqueued_frames=enqueued_frames+1;
    end
    if(codec_valid && codec_ready) begin
        if(dma_frames>=FRAME_COUNT) begin $display("[TB_FAIL] extra DMA record");$finish;end
        if(codec_record[95:64]!==dma_frames || codec_indices!==dma_expected[dma_frames] ||
           codec_record[127:96]!==dma_timestamps[dma_frames] || codec_keep!==16'hffff || codec_last!==1'b1) begin
            $display("[ERROR] DMA frame=%0d record=%h expected_indices=%h",dma_frames,codec_record,dma_expected[dma_frames]);
            errors=errors+1;
        end else $display("[DMA_PASS] frame=%0d packed=%h timestamp_us=%0d",dma_frames,codec_indices,codec_record[127:96]);
        $fdisplay(dma_file,"%032h",codec_record);
        $fdisplay(dma_timing_file,"%0d,%0d,%0d,%0d,%0d",dma_frames,
            (dma_enqueue_cycles[dma_frames]-dma_input_cycles[dma_frames])*CLOCK_PERIOD_NS,
            (cycle_count-dma_enqueue_cycles[dma_frames])*CLOCK_PERIOD_NS,
            (cycle_count-dma_input_cycles[dma_frames])*CLOCK_PERIOD_NS,codec_record[127:96]);
        dma_frames=dma_frames+1;
    end
end

task load_independent_frame;
    integer i;
    begin
        // 清的是TB计数器，不是硬件状态；全局errors、Boot计数及DMA计数保留。
        compare_count=0;pcm_compare_count=0;pcm_done_cycle=-1;
        encoder_done_cycle=-1;codec_enqueued_cycle=-1;prefetch_pending_layer=-1;
        projection_checks=0;score_checks=0;update_checks=0;stage_checks=0;rvq_stage_errors=0;
        for(i=0;i<63;i=i+1) begin
            actual_output_bytes[i]=0;expected_output_bytes[i]=0;
            compute_start_cycle[i]=-1;compute_done_cycle[i]=-1;
            prefetch_start_cycle[i]=-1;prefetch_done_cycle[i]=-1;prefetch_blocks[i]=0;
            first_weight_read_cycle[i]=-1;weight_sram_read_count[i]=0;layer_start_errors[i]=errors;
            local_a_count[i]=0;local_b_count[i]=0;act_reads[i]=0;parameter_reads[i]=0;
            g2l_active_cycles[i]=0;act_stall_cycles[i]=0;weight_stall_cycles[i]=0;
            mesh_load_cycles[i]=0;first_mesh_load[i]=-1;last_mesh_load[i]=-1;
        end
        $sformat(frame_dir,"tb/data/encoder_rvq_multiframe/frame_%0d%0d%0d",
                 frame_number/100,(frame_number/10)%10,frame_number%10);
        $sformat(frame_path,"%0s/pcm16.hex",frame_dir);require_frame_file(frame_path);$readmemh(frame_path,pcm_memory);
        $sformat(frame_path,"%0s/pcm_activation_int16.hex",frame_dir);require_frame_file(frame_path);$readmemh(frame_path,expected_pcm_activation);
        // 未使用槽位设X，错误地址不能误用上一帧残留Golden蒙混通过。
        for(i=0;i<GOLDEN_BYTES;i=i+1) golden_layers[i]=16'hxxxx;
        $sformat(frame_path,"%0s/golden_layers.mem",frame_dir);require_frame_file(frame_path);$readmemh(frame_path,golden_layers);
        $sformat(frame_path,"%0s/projection.hex",frame_dir);require_frame_file(frame_path);$readmemh(frame_path,expected_projection);
        $sformat(frame_path,"%0s/stage_input.hex",frame_dir);require_frame_file(frame_path);$readmemh(frame_path,expected_stage_input);
        $sformat(frame_path,"%0s/stage_next.hex",frame_dir);require_frame_file(frame_path);$readmemh(frame_path,expected_stage_next);
        $sformat(frame_path,"%0s/subtract.hex",frame_dir);require_frame_file(frame_path);$readmemh(frame_path,expected_subtract);
        $sformat(frame_path,"%0s/selected.hex",frame_dir);require_frame_file(frame_path);$readmemh(frame_path,expected_selected);
        $sformat(frame_path,"%0s/indices.hex",frame_dir);require_frame_file(frame_path);$readmemh(frame_path,expected_indices);
        $sformat(frame_path,"%0s/scores.hex",frame_dir);require_frame_file(frame_path);$readmemh(frame_path,expected_scores);
        $sformat(frame_path,"%0s/packed.hex",frame_dir);require_frame_file(frame_path);$readmemh(frame_path,expected_packed);
    end
endtask

task wait_multiframe_enqueued;
    integer waited;
    begin
        waited=0;
        while(enqueued_frames<=frame_number && waited<100000) begin @(negedge clk);waited=waited+1;end
        if(enqueued_frames!=frame_number+1) begin $display("[TB_FAIL] enqueue timeout frame=%0d",frame_number);$finish;end
        if(projection_checks!=32 || score_checks!=1280 || update_checks!=288 || stage_checks!=9 ||
           backend_writes!=1536 || codec_vector!==512'd0) begin
            $display("[ERROR] frame=%0d backend coverage/vector",frame_number);errors=errors+1;
        end
        $display("[ENQUEUE_TIMING] frame=%0d pcm_to_record_ns=%0d backend_ns=%0d",frame_number,
            (codec_enqueued_cycle-frame_start_cycle)*CLOCK_PERIOD_NS,
            (codec_enqueued_cycle-encoder_done_cycle)*CLOCK_PERIOD_NS);
    end
endtask

task finish_multiframe_dma;
    integer waited;
    begin
        waited=0;
        while(dma_frames<FRAME_COUNT && waited<10000) begin @(negedge clk);waited=waited+1;end
        repeat(5) @(negedge clk);
        if(dma_frames!=FRAME_COUNT || enqueued_frames!=FRAME_COUNT || codec_level!=0 || codec_valid ||
           max_queue_level<3 || dma_stalled_cycles==0 || cs_count!=3 || boot_writes!=6912 || backend_writes!=1536) begin
            $display("[ERROR] multiframe count/queue/boot mismatch enqueue=%0d dma=%0d level=%0d max=%0d CS=%0d",
                     enqueued_frames,dma_frames,codec_level,max_queue_level,cs_count);errors=errors+1;
        end
        $fclose(dma_file);$fclose(dma_timing_file);
        $display("[MULTIFRAME_SUMMARY] frames=%0d DMA=%0d max_queue=%0d stalled_cycles=%0d Flash_CS=%0d reset_once=1",
                 enqueued_frames,dma_frames,max_queue_level,dma_stalled_cycles,cs_count);
    end
endtask
