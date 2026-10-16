# Generate deterministic data, compile the complete RTL and simulate one PCM frame.
param(
    [switch] $RealSpi,
    [switch] $OnchipOnly
)

$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$simBin = 'E:\modelsim_24\win64pe'
$workLib = Join-Path $projectRoot 'build\full_encoder_frame_work'
$logDir = Join-Path $projectRoot 'logs'
$simLogName = if ($OnchipOnly) {
    'full_encoder_onchip.log'
} elseif ($RealSpi) {
    'full_encoder_frame_real_spi.log'
} else {
    'full_encoder_frame.log'
}
$simLog = Join-Path $logDir $simLogName

New-Item -ItemType Directory -Force -Path $logDir | Out-Null

function Quote-NativeArgument {
    param([Parameter(Mandatory = $true)] [string] $Value)
    return '"' + $Value + '"'
}

function Invoke-ModelSimTool {
    param(
        [Parameter(Mandatory = $true)] [string] $Tool,
        [Parameter(Mandatory = $true)] [string[]] $Arguments,
        [Parameter(Mandatory = $true)] [string] $FailureMessage
    )
    $toolPath = Join-Path $simBin $Tool
    if (-not (Test-Path -LiteralPath $toolPath)) {
        throw "ModelSim executable not found: $toolPath"
    }
    $process = Start-Process -FilePath $toolPath `
        -ArgumentList $Arguments -NoNewWindow -Wait -PassThru
    if ($process.ExitCode -ne 0) {
        throw "$FailureMessage (exit code $($process.ExitCode))"
    }
}

Push-Location $projectRoot
try {
    if ($RealSpi -and $OnchipOnly) {
        throw 'RealSpi and OnchipOnly cannot be enabled together'
    } elseif ($OnchipOnly) {
        Write-Host 'Mode: ON-CHIP First/A/B Weight SRAM full-frame regression'
    } elseif ($RealSpi) {
        Write-Host 'Mode: REAL SPI bit-level simulation (very slow)'
    } else {
        Write-Host 'Mode: FAST full-frame numerical regression + analytical SPI timing'
    }
    & python (Join-Path $PSScriptRoot 'generate_full_encoder_test_data.py')
    if ($LASTEXITCODE -ne 0) {
        throw 'Full encoder test-data generation failed'
    }

    if (-not (Test-Path -LiteralPath (Join-Path $workLib '_info'))) {
        Invoke-ModelSimTool -Tool 'vlib.exe' `
            -Arguments @((Quote-NativeArgument $workLib)) `
            -FailureMessage 'vlib failed'
    }

    $svSources = @(
        'third_party\opentitan_sram\prim_ram_1p_pkg.sv',
        'third_party\opentitan_sram\prim_ram_2p_pkg.sv',
        'third_party\opentitan_sram\prim_ram_1r1w_pkg.sv',
        'third_party\opentitan_sram\prim_ram_1p.sv',
        'third_party\opentitan_sram\prim_ram_1r1w.sv',
        'rtl\opentitan_sram_1p_adapter.sv',
        'rtl\opentitan_sram_1r1w_adapter.sv'
    ) | ForEach-Object { Join-Path $projectRoot $_ }
    $svArguments = @(
        '-sv', '-work', (Quote-NativeArgument $workLib),
        '-l', (Quote-NativeArgument (Join-Path $logDir 'full_encoder_sram_compile.log'))
    )
    $svArguments += $svSources | ForEach-Object { Quote-NativeArgument $_ }
    Invoke-ModelSimTool -Tool 'vlog.exe' -Arguments $svArguments `
        -FailureMessage 'OpenTitan SRAM compile failed'

    $vSources = @(
        'rtl\flash\spi_clgen.v',
        'rtl\flash\spi_shift.v',
        'rtl\flash\spi_top.v',
        'rtl\flash\apb_single_master.v',
        'rtl\flash\flash_weight_burst_loader.v',
        'rtl\flash\qspi_weight_boot_loader.v',
        'rtl\weight\encoder_weight_hierarchy.v',
        'rtl\flash\weight_cache_ab.v',
        'rtl\DDRphy\octal_ddr_async_fifo.v',
        'rtl\DDRphy\octal_ddr_16to256_packer.v',
        'rtl\DMA\weight_stream_dma.v',
        'rtl\DMA\weight_cache_ab_octal.v',
        'tb\weight_cache_ab_fast_model.v',
        'rtl\PE.v', 'rtl\Systolic_Array.v',
        'rtl\input_skew_4lane.v', 'rtl\output_deskew_8lane.v',
        'rtl\mesh_core_4x8_systolic.v', 'rtl\mesh_adapter.v',
        'rtl\conv1d_config_latch.v', 'rtl\conv1d_agu.v',
        'rtl\activation_reader_4lane.v', 'rtl\conv1d_weight_agu.v',
        'rtl\weight_loader.v', 'rtl\accumulator_8lane.v',
        'rtl\postprocess_8lane.v', 'rtl\elu_8lane.v',
        'rtl\output_writer_8lane.v', 'rtl\clock\mesh_icg.v','rtl\conv1d_controller.v',
        'rtl\depthwise_conv1d_controller.v',
        'rtl\conv1d_engine_top.v', 'rtl\conv1d_engine_flash_top.v',
        'rtl\activation_buffer_pingpong.v',
        'rtl\soundstream_encoder_layer_rom.v',
        'rtl\soundstream_encoder_scheduler.v',
        'rtl\residual_capture.v', 'rtl\residual_add_8lane.v',
        'rtl\residual_scratchpad.v', 'rtl\soundstream_encoder_top.v',
        'rtl\pcm16_to_int8_quantizer.v', 'rtl\pcm16_input_frontend.v',
        'rtl\flash\W25Q128JVxIM.v',
        'tb\tb_soundstream_encoder_full_frame.v'
    ) | ForEach-Object { Join-Path $projectRoot $_ }

    $vArguments = @(
        '-work', (Quote-NativeArgument $workLib), '-vlog01compat',
        '+incdir+rtl/flash',
        '+define+MEM_FILENAME=\"tb/data/full_encoder/weights.mem\"',
        '+define+SECSI_FILENAME=\"tb/data/flash/SECSI.TXT\"',
        '+define+SFDP_FILENAME=\"tb/data/flash/SFDP.TXT\"',
        '+define+SREG_FILENAME=\"tb/data/flash/SREG.TXT\"',
        '-l', (Quote-NativeArgument (Join-Path $logDir 'full_encoder_rtl_compile.log'))
    )
    if ($RealSpi) {
        $vArguments += '+define+FULL_ENCODER_REAL_SPI'
    }
    if ($OnchipOnly) {
        $vArguments += '+define+FULL_ENCODER_ONCHIP_ONLY'
    }
    $vArguments += $vSources | ForEach-Object { Quote-NativeArgument $_ }
    Invoke-ModelSimTool -Tool 'vlog.exe' -Arguments $vArguments `
        -FailureMessage 'Full encoder RTL compile failed'

    $simArguments = @(
        '-c', '-lib', (Quote-NativeArgument $workLib),
        'tb_soundstream_encoder_full_frame',
        '-l', (Quote-NativeArgument $simLog),
        '-do', '"run -all; quit -f"'
    )
    Invoke-ModelSimTool -Tool 'vsim.exe' -Arguments $simArguments `
        -FailureMessage "Full encoder simulation failed; see $simLog"

    Write-Host "Numerical and timing log: $simLog"
    if ($OnchipOnly) {
        Write-Host "Per-layer timing CSV: $(Join-Path $logDir 'full_encoder_onchip_timing.csv')"
    } else {
        Write-Host "Per-layer timing CSV: $(Join-Path $logDir 'full_encoder_timing.csv')"
    }
} finally {
    Pop-Location
}
