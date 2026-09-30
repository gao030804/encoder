param(
    [switch] $CompileOnly,
    [string] $Wav = '',
    [string] $PcmDir = '',
    [switch] $DerivedInputs,
    [int] $StartSample = 0,
    [string] $RvqDir = 'E:\lyra\rvq',
    [string] $ProjectionDir = 'E:\lyra\rvq'
)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$bin='E:\modelsim_24\win64pe'
Push-Location $root
try {
    if (($Wav -and $PcmDir) -or ($DerivedInputs -and ($Wav -or $PcmDir))) {
        throw 'Choose only one input: -Wav, -PcmDir, or -DerivedInputs'
    }
    # 默认使用用户提供的10帧真实PCM；只有显式-DerivedInputs才使用派生数据。
    if (-not $Wav -and -not $PcmDir -and -not $DerivedInputs) { $PcmDir=Join-Path $root 'pcm' }
    if ($Wav -and -not (Test-Path -LiteralPath $Wav -PathType Leaf)) { throw "WAV file not found: $Wav" }
    if ($PcmDir) {
        if ($StartSample -ne 0) { throw '-StartSample is only applicable to WAV input' }
        for ($i=0;$i -lt 10;$i++) {
            $pcmFile=Join-Path $PcmDir ('frame_{0:d2}.pcm' -f $i)
            if (-not (Test-Path -LiteralPath $pcmFile -PathType Leaf)) { throw "PCM file not found: $pcmFile" }
            if ((Get-Item -LiteralPath $pcmFile).Length -ne 640) { throw "PCM must contain 640 bytes: $pcmFile" }
        }
    }
    & ./scripts/run_flash_encoder_rvq_modelsim.ps1 -CompileOnly
    if ($CompileOnly) { return }
    & python scripts/prepare_encoder_rvq_frame.py --rvq-dir $RvqDir --projection-dir $ProjectionDir
    if ($LASTEXITCODE -ne 0) { throw 'Common Flash image preparation failed' }
    $prepare=@('scripts/prepare_encoder_rvq_multiframe.py','--rvq-dir',$RvqDir,'--projection-dir',$ProjectionDir,'--start-sample',$StartSample)
    if ($Wav) { $prepare+=@('--wav',$Wav) }
    elseif ($PcmDir) { $prepare+=@('--pcm-dir',$PcmDir) }
    else { Write-Host 'TEST INPUT: 10 transformed versions of existing PCM frame; not consecutive recorded audio' }
    & python @prepare
    if ($LASTEXITCODE -ne 0) { throw 'Multi-frame Golden preparation failed' }
    $manifestPath=Join-Path $root 'tb/data/encoder_rvq_multiframe/manifest.json'
    $manifest=Get-Content -LiteralPath $manifestPath -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($PcmDir) { $expectedMode='raw_pcm_independent_frames' }
    elseif ($Wav) { $expectedMode='wav_independent_frames' }
    else { $expectedMode='derived_single_frame_test_vectors' }
    if ($manifest.source.mode -ne $expectedMode -or $manifest.frame_count -ne 10) {
        throw "Input provenance mismatch: expected $expectedMode, got $($manifest.source.mode)"
    }
    if ($PcmDir) {
        $combined=Join-Path $PcmDir 'frames_00_to_09.pcm'
        if (Test-Path -LiteralPath $combined -PathType Leaf) {
            $actualHash=(Get-FileHash -LiteralPath $combined -Algorithm SHA256).Hash.ToLowerInvariant()
            if ($actualHash -ne $manifest.source.sha256) { throw 'PCM changed after Golden generation' }
        }
    }
    Write-Host "[INPUT_PROVENANCE] mode=$expectedMode sha256=$($manifest.source.sha256)"
    & "$bin/vsim.exe" -c -lib build/flash_encoder_frame_work tb_flash_encoder_rvq_full_frame `
        -gFRAME_COUNT=10 -l logs/encoder_rvq_multiframe.log -do 'run -all; quit -f'
    if ($LASTEXITCODE -ne 0) { throw 'Multi-frame simulation failed' }
    $result=[IO.File]::ReadAllText((Join-Path $root 'logs/encoder_rvq_multiframe.log'))
    $expectedSourceLine="[INPUT_SOURCE] mode=$expectedMode sha256=$($manifest.source.sha256)"
    $expectedClockLine='[CLOCK_CONFIG] system_hz=50000000 qspi_hz=25000000 period_ns=20'
    if ($result -notmatch '\[TB_PASS\]' -or $result -notmatch [regex]::Escape($expectedSourceLine) -or
        $result -notmatch [regex]::Escape($expectedClockLine) -or
        $result -match '\[\w*FAIL\]|\[ERROR\]|\*\* (Error|Fatal):') {
        throw 'Multi-frame RTL checks failed'
    }
    foreach ($item in @(@('LAYER_PASS',630),@('RVQ_STAGE_PASS',90),@('FRAME_PASS',10),@('DMA_PASS',10),@('LATENT_PASS',10))) {
        if ([regex]::Matches($result,('\['+$item[0]+'\]')).Count -ne $item[1]) { throw "Missing coverage: $($item[0])" }
    }
    & python scripts/decode_rvq_records.py --records logs/encoded_records_multiframe.hex --rvq-dir $RvqDir `
        --expect-golden tb/data/encoder_rvq_multiframe/output_all.hex --output logs/rvq_software_multiframe.json
    if ($LASTEXITCODE -ne 0) { throw 'Multi-frame software vector reconstruction failed' }
    Write-Host '10 independent frames PASS: one boot, 630 layers, 90 RVQ stages, 10 ordered DMA records (not streaming-history verification)'
} finally { Pop-Location }
