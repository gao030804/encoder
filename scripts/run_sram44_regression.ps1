param(
    [switch] $VerifyOnly,
    [switch] $SkipUnits,
    [string] $BaselineDir='logs/sram44_baseline_50mhz'
)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    $manifest=Get-Content -LiteralPath 'tb/data/encoder_rvq_multiframe/manifest.json' -Raw | ConvertFrom-Json
    if($manifest.source.mode -ne 'raw_pcm_independent_frames' -or $manifest.frame_count -ne 10){throw 'Expected ten independent real PCM frames'}
    foreach($frame in $manifest.frames){
        $pcmPath='pcm/frame_{0:D2}.pcm' -f [int]$frame.frame
        if((Get-FileHash -LiteralPath $pcmPath -Algorithm SHA256).Hash.ToLowerInvariant() -ne $frame.pcm_sha256){throw "PCM hash mismatch: $pcmPath"}
    }
    if((Get-FileHash -LiteralPath 'pcm/frames_00_to_09.pcm' -Algorithm SHA256).Hash.ToLowerInvariant() -ne $manifest.source.sha256){throw 'Combined PCM hash mismatch'}
    Write-Host '[PCM_PROVENANCE_PASS] actual ten PCM files match Golden manifest'
    if(-not $VerifyOnly){
        $runner=Join-Path $PSScriptRoot 'run_qor_p0_regression.ps1'
        if(-not $SkipUnits){& $runner -Phase Units -ClockMHz 50 -LogTag sram44}
        & $runner -Phase Full -Frames 10 -ClockMHz 50 -LogTag sram44 -SramTrimChecks
        & $runner -Phase Full -Frames 10 -ClockMHz 30 -LogTag sram44 -SramTrimChecks
    }
    $unitTops=@('tb_mesh_scan_port_compat','tb_sram_banking','tb_sram_trim_ports',
        'tb_encoder_weight_hierarchy','tb_residual_guard_optimization','tb_residual_scale_align',
        'tb_postprocess_guard_sticky','tb_postprocess_pipeline','tb_postprocess_depthwise',
        'tb_rvq_backend_arithmetic','tb_qor_p0_projection','tb_rvq_backend_update',
        'tb_rvq_backend_vector_overflow','tb_qor_p0_post_stream')
    foreach($unit in $unitTops){
        $unitFile="logs/sram44_50mhz/$unit.log"
        $unitLog=[IO.File]::ReadAllText((Resolve-Path -LiteralPath $unitFile).Path)
        if($unitLog -notmatch '\[TB_PASS\]' -or $unitLog -match '\[\w*FAIL\]|\[ERROR\]|ERROR:|FATAL:'){
            throw "Unit regression did not pass: $unitFile"
        }
    }
    $results=@()
    $recordReference=$null
    $layerReference=$null
    $cycleReference=$null
    $baselineLog=Join-Path $BaselineDir 'tb_qor_p0_full_frame.log'
    $baselineRecords=Join-Path $BaselineDir 'encoded_records_multiframe.hex'
    if(Test-Path -LiteralPath $baselineLog){
        $baselineText=[IO.File]::ReadAllText((Resolve-Path -LiteralPath $baselineLog).Path)
        if($baselineText -notmatch '\[TB_PASS\]' -or $baselineText -match '\[\w*FAIL\]'){
            throw 'Baseline did not pass'
        }
        $layerReference=@([regex]::Matches($baselineText,'(?m)^\[LAYER_PASS\][^\r\n]*') | ForEach-Object {$_.Value})
        $cycleReference=@([regex]::Matches($baselineText,'\[QOR_DEADLINE\] frame=(\d+) clock_hz=\d+ cycles=(\d+)') | ForEach-Object {$_.Groups[2].Value})
        # Record = timestamp[31:0], frame_id[31:0], packed64; compare low 96 bits.
        $recordReference=@(Get-Content -LiteralPath $baselineRecords | Where-Object {$_ -match '^[0-9a-fA-F]{32}$'} | ForEach-Object {$_.Substring(8).ToLowerInvariant()})
        if($layerReference.Count -ne 630 -or $cycleReference.Count -ne 10 -or $recordReference.Count -ne 10){throw 'Baseline coverage incomplete'}
    }
    foreach($mhz in @(50,30)){
        $dir="logs/sram44_${mhz}mhz"
        $logFile=Join-Path $dir 'tb_sram_trim_full_frame.log'
        $log=[IO.File]::ReadAllText((Resolve-Path -LiteralPath $logFile).Path)
        if($log -notmatch '\[TB_PASS\]' -or $log -match '\[\w*FAIL\]|\[ERROR\]|ERROR:|FATAL:'){throw "Failed full regression: $logFile"}
        foreach($item in @(@('LAYER_PASS',630),@('RVQ_STAGE_PASS',90),@('LATENT_PASS',10),@('FRAME_PASS',10),@('DMA_PASS',10),@('QOR_DEADLINE',10))){
            if([regex]::Matches($log,('\['+$item[0]+'\]')).Count -ne $item[1]){throw "Missing coverage $($item[0]): $logFile"}
        }
        if($log -notmatch '\[SRAM_TRIM_CONFIG\][^\r\n]*target_total_macros=75 saved_KiB=44' -or
           $log -notmatch '\[SRAM_TRIM_PASS\] frames=10 global_max=6911 activation_row_max=1279 residual_word_max=1279[^\r\n]*saved_bytes=45056'){
            throw "Missing capacity/address proof: $logFile"
        }
        # Ensure the actual compiled local storage sources still match disk.
        $sources=Import-Csv -LiteralPath (Join-Path $dir 'source_hashes.csv')
        foreach($relative in @('rtl/weight/encoder_weight_hierarchy.v','rtl/conv1d_engine_flash_top.v',
            'rtl/soundstream_encoder_top.v','rtl/soundstream_encoder_rvq_w8a16_top.v',
            'rtl/activation_buffer_pingpong.v','rtl/residual_scratchpad.v',
            'rtl/residual_capture.v','rtl/opentitan_sram_1p_adapter.sv')){
            $path=(Resolve-Path -LiteralPath $relative).Path
            $entry=@($sources | Where-Object {$_.Path -eq $path})
            if($entry.Count -ne 1 -or $entry[0].Hash -ne (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash){
                throw "Compiled SRAM RTL hash mismatch: $relative"
            }
        }
        $records=@(Get-Content -LiteralPath (Join-Path $dir 'encoded_records_multiframe.hex') | Where-Object {$_ -match '^[0-9a-fA-F]{32}$'} | ForEach-Object {$_.Substring(8).ToLowerInvariant()})
        $layers=@([regex]::Matches($log,'(?m)^\[LAYER_PASS\][^\r\n]*') | ForEach-Object {$_.Value})
        $cycles=@([regex]::Matches($log,'\[QOR_DEADLINE\] frame=(\d+) clock_hz=\d+ cycles=(\d+)') | ForEach-Object {$_.Groups[2].Value})
        if($records.Count -ne 10){throw 'Expected exactly ten DMA records'}
        if($null -eq $recordReference){$recordReference=$records;$layerReference=$layers;$cycleReference=$cycles}
        if(($records -join ',') -ne ($recordReference -join ',')){throw 'Frame IDs or packed64 changed'}
        if(($layers -join "`n") -ne ($layerReference -join "`n")){throw 'Per-layer traffic/cycles changed'}
        if(($cycles -join ',') -ne ($cycleReference -join ',')){throw 'Frame cycles changed'}
        $worst=($cycles | ForEach-Object {[int]$_} | Measure-Object -Maximum).Maximum
        $ms=$worst/($mhz*1000.0)
        if($ms -ge 20.0){throw '20 ms frame deadline exceeded'}
        $results+=[pscustomobject]@{
            system_mhz=$mhz; qspi_mhz=$mhz/2.0; frames=10; layer_checks=630; rvq_checks=90;
            dma_checks=10; worst_cycles=$worst; encoding_ms=$ms; rtf=$ms/20.0;
            saved_bytes=45056; main_sram_kib=340.5; target_macro_count=75;
            compiled_storage_hashes_verified=$true; log=$logFile
        }
    }
    $summary=[pscustomobject]@{
        simulator='XSIM 2018.3'; independent_real_pcm=$true; pcm_sha256=$manifest.source.sha256; unit_tests_passed=$unitTops.Count;
        baseline_compared=(Test-Path -LiteralPath $baselineLog);
        frame_id_packed64_equal=$true; per_layer_cycles_equal=$true;
        physical_synthesis_verified=$false; leakage_reduction_measured=$false;
        results=$results
    }
    $summary | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath 'logs/sram44_validation_summary.json' -Encoding UTF8
    $results | Format-Table system_mhz,qspi_mhz,frames,worst_cycles,encoding_ms,rtf
    Write-Host '[SRAM44_REGRESSION_PASS] both clocks, Golden, bounds, source hashes, frame_id and packed64 checked'
} finally {Pop-Location}
