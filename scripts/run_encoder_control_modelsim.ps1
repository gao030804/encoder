# Compile the complete encoder RTL, then run the residual-add and scheduler tests.
$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$simBin = 'E:\modelsim_24\win64pe'
$workLib = Join-Path $projectRoot 'build\encoder_control_work'
$logDir = Join-Path $projectRoot 'logs'

New-Item -ItemType Directory -Force -Path $logDir | Out-Null

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

function Quote-NativeArgument {
    param([Parameter(Mandatory = $true)] [string] $Value)
    return '"' + $Value + '"'
}

if (-not (Test-Path -LiteralPath (Join-Path $workLib '_info'))) {
    Invoke-ModelSimTool -Tool 'vlib.exe' `
        -Arguments @((Quote-NativeArgument $workLib)) `
        -FailureMessage 'vlib failed'
}

$systemVerilogSources = @(
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
    '-l', (Quote-NativeArgument (Join-Path $logDir 'encoder_control_sram_compile.log'))
)
$svArguments += $systemVerilogSources | ForEach-Object { Quote-NativeArgument $_ }
Invoke-ModelSimTool -Tool 'vlog.exe' -Arguments $svArguments `
    -FailureMessage 'OpenTitan SRAM model compile failed'

$verilogSources = @(
    'rtl\flash\spi_clgen.v',
    'rtl\flash\spi_shift.v',
    'rtl\flash\spi_top.v',
    'rtl\flash\apb_single_master.v',
    'rtl\flash\flash_weight_burst_loader.v',
    'rtl\flash\weight_cache_ab.v',
    'rtl\DDRphy\octal_ddr_async_fifo.v',
    'rtl\DDRphy\octal_ddr_16to256_packer.v',
    'rtl\DMA\weight_stream_dma.v',
    'rtl\DMA\weight_cache_ab_octal.v',
    'rtl\PE.v',
    'rtl\Systolic_Array.v',
    'rtl\input_skew_4lane.v',
    'rtl\output_deskew_8lane.v',
    'rtl\mesh_core_4x8_systolic.v',
    'rtl\mesh_adapter.v',
    'rtl\conv1d_config_latch.v',
    'rtl\conv1d_agu.v',
    'rtl\activation_reader_4lane.v',
    'rtl\conv1d_weight_agu.v',
    'rtl\weight_loader.v',
    'rtl\accumulator_8lane.v',
    'rtl\postprocess_8lane.v',
    'rtl\elu_8lane.v',
    'rtl\output_writer_8lane.v',
    'rtl\clock\mesh_icg.v','rtl\conv1d_controller.v',
    'rtl\depthwise_conv1d_controller.v',
    'rtl\conv1d_engine_top.v',
    'rtl\conv1d_engine_flash_top.v',
    'rtl\activation_buffer_pingpong.v',
    'rtl\soundstream_encoder_layer_rom.v',
    'rtl\soundstream_encoder_scheduler.v',
    'rtl\residual_capture.v',
    'rtl\residual_add_8lane.v',
    'rtl\residual_scratchpad.v',
    'rtl\soundstream_encoder_top.v',
    'tb\tb_residual_add_8lane.v',
    'tb\tb_soundstream_encoder_scheduler.v'
) | ForEach-Object { Join-Path $projectRoot $_ }

$vArguments = @(
    '-work', (Quote-NativeArgument $workLib), '-vlog01compat',
    '-l', (Quote-NativeArgument (Join-Path $logDir 'encoder_control_rtl_compile.log'))
)
$vArguments += $verilogSources | ForEach-Object { Quote-NativeArgument $_ }
Invoke-ModelSimTool -Tool 'vlog.exe' -Arguments $vArguments `
    -FailureMessage 'Encoder Verilog-2001 compile failed'

foreach ($testbench in @('tb_residual_add_8lane', 'tb_soundstream_encoder_scheduler')) {
    $testLog = Join-Path $logDir ($testbench + '.log')
    $simArguments = @(
        '-c', '-lib', (Quote-NativeArgument $workLib), $testbench,
        '-l', (Quote-NativeArgument $testLog), '-do', '"run -all; quit -f"'
    )
    Invoke-ModelSimTool -Tool 'vsim.exe' -Arguments $simArguments `
        -FailureMessage "Simulation $testbench failed; see $testLog"
    Write-Host "Simulation log: $testLog"
}
