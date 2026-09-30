// 继承Encoder逐层scoreboard；此处只加载期望值，不初始化DUT任何存储。
wire codec_valid;
reg codec_ready=0;
wire [63:0] codec_indices;
wire [511:0] codec_vector;
wire [127:0] codec_record;
wire [15:0] codec_keep;
wire codec_last,codec_enqueued;
wire [8:0] codec_level;
reg [31:0] expected_timestamp;
integer codec_enqueued_cycle=-1;
always @(posedge clk) if(rst_n) begin
    if(encoder_start && encoder_start_ready) expected_timestamp=cycle_count/CYCLES_PER_US;
    if(codec_enqueued) codec_enqueued_cycle=cycle_count;
end
reg [15:0] expected_projection[0:31], expected_rvq_output[0:31];
reg [15:0] expected_stage_input[0:287], expected_stage_next[0:287];
reg [16:0] expected_subtract[0:287];
reg [7:0] expected_selected[0:287], expected_indices[0:8];
reg [31:0] expected_scores[0:2303];
reg [63:0] expected_packed[0:0];
reg [7:0] test_mode[0:0];
reg [255:0] backend_expected[0:1535];
integer projection_checks=0,score_checks=0,update_checks=0,stage_checks=0,backend_writes=0;
integer rvq_stage_errors=0,rvq_j,rvq_offset,codec_done_cycle;
initial begin
    $readmemh("tb/data/flash_encoder_rvq/projection.hex",expected_projection);
    $readmemh("tb/data/flash_encoder_rvq/output.hex",expected_rvq_output);
    $readmemh("tb/data/flash_encoder_rvq/stage_input.hex",expected_stage_input);
    $readmemh("tb/data/flash_encoder_rvq/stage_next.hex",expected_stage_next);
    $readmemh("tb/data/flash_encoder_rvq/subtract.hex",expected_subtract);
    $readmemh("tb/data/flash_encoder_rvq/selected.hex",expected_selected);
    $readmemh("tb/data/flash_encoder_rvq/indices.hex",expected_indices);
    $readmemh("tb/data/flash_encoder_rvq/scores.hex",expected_scores);
    $readmemh("tb/data/flash_encoder_rvq/packed.hex",expected_packed);
    $readmemh("tb/data/flash_encoder_rvq/backend_blocks.hex",backend_expected);
    $readmemh("tb/data/flash_encoder_rvq/test_mode.hex",test_mode);
    #1;
    if(test_mode[0]==1) $display("[TEST_MODE] SYNTHETIC projection smoke; real RVQ books; NOT real complete model verification");
    else $display("[TEST_MODE] Real exported projection and RVQ parameters");
end
always @(posedge clk) if(rst_n && u_dut.boot_aux_write) begin
    if(u_dut.boot_aux_index!==backend_writes || u_dut.boot_aux_data!==backend_expected[backend_writes]) begin
        $display("[ERROR] RVQ Boot data/order block=%0d",backend_writes);errors=errors+1;
    end
    backend_writes=backend_writes+1;
end

// NBA后检查观察口，避免测试平台采样竞态。
always @(negedge clk) if(rst_n) begin
    if(u_dut.u_backend.projection_valid) begin
        if(u_dut.u_backend.projection_data !== expected_projection[u_dut.u_backend.projection_channel]) begin
            $display("[ERROR] projection channel=%0d",u_dut.u_backend.projection_channel);errors=errors+1;
        end
        projection_checks=projection_checks+1;
        if(projection_checks==32) $display("[PROJECTION_CHECK] compared all 32 INT16 outputs");
    end
    if(u_dut.u_backend.score_valid) begin
        if(u_dut.u_backend.trace_score !== expected_scores[256*u_dut.u_backend.trace_stage+u_dut.u_backend.trace_candidate]) begin
            $display("[ERROR] score stage=%0d candidate=%0d",u_dut.u_backend.trace_stage,u_dut.u_backend.trace_candidate);
            errors=errors+1;rvq_stage_errors=rvq_stage_errors+1;
        end
        score_checks=score_checks+1;
    end
    if(u_dut.u_backend.update_valid) begin
        rvq_offset=32*u_dut.u_backend.trace_stage+u_dut.u_backend.trace_dimension;
        if(u_dut.u_backend.trace_residual !== expected_stage_input[rvq_offset] ||
           u_dut.u_backend.trace_codeword !== expected_selected[rvq_offset] ||
           u_dut.u_backend.trace_subtract !== expected_subtract[rvq_offset] ||
           u_dut.u_backend.trace_next !== expected_stage_next[rvq_offset]) begin
            $display("[ERROR] residual update stage=%0d dimension=%0d",u_dut.u_backend.trace_stage,u_dut.u_backend.trace_dimension);
            errors=errors+1;rvq_stage_errors=rvq_stage_errors+1;
        end
        update_checks=update_checks+1;
    end
    if(u_dut.u_backend.stage_valid) begin
        if(u_dut.u_backend.trace_index !== expected_indices[u_dut.u_backend.trace_stage]) begin
            $display("[ERROR] index stage=%0d",u_dut.u_backend.trace_stage);errors=errors+1;rvq_stage_errors=rvq_stage_errors+1;
        end
        if(rvq_stage_errors==0) $display("[RVQ_STAGE_PASS] frame=%0d stage=%0d index=%0d score/residual/codeword match",frame_number,u_dut.u_backend.trace_stage,u_dut.u_backend.trace_index);
        else $display("[RVQ_STAGE_FAIL] frame=%0d stage=%0d",frame_number,u_dut.u_backend.trace_stage);
        stage_checks=stage_checks+1;rvq_stage_errors=0;
    end
end

task wait_rvq_complete;
    integer waited,n,record_file;
    reg [63:0] held_indices;
    reg [511:0] held_vector;
    reg [127:0] held_record;
    begin
        waited=0;
        while(!codec_valid && waited<100000) begin @(negedge clk);waited=waited+1;end
        if(!codec_valid) begin $display("[TB_FAIL] RVQ timeout");$finish;end
        codec_done_cycle=cycle_count;
        if(codec_indices !== expected_packed[0]) begin $display("[ERROR] packed indices expected=%h actual=%h",expected_packed[0],codec_indices);errors=errors+1;end
        if(codec_vector!==512'd0) begin $display("[ERROR] vector debug should be disabled");errors=errors+1;end
        if(codec_record[95:64]!==32'd0 || codec_record[127:96]!==expected_timestamp ||
           codec_keep!==16'hffff || codec_last!==1'b1 || codec_level!==9'd1 || codec_enqueued_cycle<0) begin
            $display("[ERROR] output SRAM record metadata/level mismatch timestamp expected=%0d actual=%0d",expected_timestamp,codec_record[127:96]);errors=errors+1;
        end
        if(projection_checks!=32 || score_checks!=1280 || update_checks!=288 || stage_checks!=9 || backend_writes!=1536) begin
            $display("[ERROR] RVQ check coverage projection=%0d score=%0d update=%0d stage=%0d boot=%0d",projection_checks,score_checks,update_checks,stage_checks,backend_writes);errors=errors+1;
        end
        held_indices=codec_indices;held_vector=codec_vector;held_record=codec_record;
        repeat(7) begin
            @(negedge clk);
            if(!codec_valid || codec_indices!==held_indices || codec_record!==held_record || codec_level!==9'd1) begin
                $display("[ERROR] result changed under backpressure");errors=errors+1;
            end
        end
        codec_ready=1;
        @(posedge clk);@(negedge clk);codec_ready=0;
        if(codec_level!==9'd0) begin $display("[ERROR] output ring did not dequeue");errors=errors+1;end
        record_file=$fopen("logs/encoded_records.hex","w");
        if(record_file==0) begin $display("[TB_FAIL] cannot write DMA record dump");$finish;end
        $fdisplay(record_file,"%032h",held_record);$fclose(record_file);
        $display("[OUTPUT_RING_CHECK] frame_id=%0d timestamp_us=%0d record=%h",held_record[95:64],held_record[127:96],held_record);
        $display("[ENQUEUE_TIMING] PCM_to_record_stored_ns=%0d",(codec_enqueued_cycle-frame_start_cycle)*CLOCK_PERIOD_NS);
        $display("[CODEC_TIMING] PCM_to_indices_ns=%0d backend_ns=%0d reset_to_indices_ns=%0d",(codec_done_cycle-frame_start_cycle)*CLOCK_PERIOD_NS,(codec_done_cycle-encoder_done_cycle)*CLOCK_PERIOD_NS,codec_done_cycle*CLOCK_PERIOD_NS);
        $display("[RVQ_RESULT] packed64=%h",held_indices);
    end
endtask
