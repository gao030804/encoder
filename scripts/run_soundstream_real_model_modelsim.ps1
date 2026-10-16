# 编译/运行“真实 .pt 参数 + 真实 .wav 音频”整帧回归。
# 在真实导出数据尚未准备好时，可使用 -CompileOnly 只检查 RTL 与 testbench 接口。
param([switch] $CompileOnly)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$bin  = 'E:\modelsim_24\win64pe'
$work = Join-Path $root 'build\soundstream_real_model_work'
$compileLog = 'logs\soundstream_real_model_compile.log'
$sramLog = 'logs\soundstream_real_model_sram_compile.log'
$simLog = 'logs\soundstream_real_model.log'

$requiredData = @(
    'tb\data\real_audio\flash_weights.hex',
    'tb\data\real_audio\parameter_bias.hex',
    'tb\data\real_audio\parameter_multiplier.hex',
    'tb\data\real_audio\parameter_shift.hex',
    'tb\data\real_audio\parameter_zero_point.hex',
    'tb\data\real_audio\projection_words.hex',
    'tb\data\real_audio\projection_multiplier.hex',
    'tb\data\real_audio\projection_shift.hex',
    'tb\data\real_audio\rvq_codebook.hex',
    'tb\data\real_audio\rvq_scale_multiplier.hex',
    'tb\data\real_audio\rvq_scale_shift.hex',
    'tb\data\real_audio\pcm_multiplier.hex',
    'tb\data\real_audio\pcm_shift.hex',
    'tb\data\real_audio\pcm_zero_point.hex',
    'tb\data\real_audio\frame_0000\pcm16.hex',
    'tb\data\real_audio\frame_0000\pcm_int8.hex',
    'tb\data\real_audio\frame_0000\golden_layers.mem',
    'tb\data\real_audio\frame_0000\latent64.hex',
    'tb\data\real_audio\frame_0000\projection32.hex',
    'tb\data\real_audio\frame_0000\rvq_indices.hex',
    'tb\data\real_audio\frame_0000\rvq_residual.hex'
)

$sources = @(
    'rtl\flash\spi_clgen.v','rtl\flash\spi_shift.v','rtl\flash\spi_top.v',
    'rtl\flash\apb_single_master.v','rtl\flash\flash_weight_burst_loader.v',
    'rtl\flash\qspi_weight_boot_loader.v','rtl\weight\encoder_weight_hierarchy.v',
    'rtl\flash\weight_cache_ab.v','rtl\DDRphy\octal_ddr_async_fifo.v',
    'rtl\DDRphy\octal_ddr_16to256_packer.v','rtl\DMA\weight_stream_dma.v',
    'rtl\DMA\weight_cache_ab_octal.v','tb\weight_cache_ab_fast_model.v',
    'rtl\PE.v','rtl\Systolic_Array.v','rtl\input_skew_4lane.v',
    'rtl\output_deskew_8lane.v','rtl\mesh_core_4x8_systolic.v','rtl\mesh_adapter.v',
    'rtl\conv1d_config_latch.v','rtl\conv1d_agu.v','rtl\activation_reader_4lane.v',
    'rtl\conv1d_weight_agu.v','rtl\weight_loader.v','rtl\accumulator_8lane.v',
    'rtl\postprocess_8lane.v','rtl\elu_8lane.v','rtl\output_writer_8lane.v',
    'rtl\clock\mesh_icg.v','rtl\conv1d_controller.v','rtl\depthwise_conv1d_controller.v',
    'rtl\conv1d_engine_top.v','rtl\conv1d_engine_flash_top.v',
    'rtl\activation_buffer_pingpong.v','rtl\soundstream_encoder_layer_rom.v',
    'rtl\soundstream_encoder_scheduler.v','rtl\residual_capture.v',
    'rtl\residual_add_8lane.v','rtl\residual_scratchpad.v',
    'rtl\soundstream_encoder_top.v','rtl\pcm16_to_int8_quantizer.v',
    'rtl\pcm16_input_frontend.v','RVQ\rvq_codebook_sram.v',
    'RVQ\rvq_scale_align_8lane.v','RVQ\rvq_distance_8lane.v',
    'RVQ\rvq_residual_update_8lane.v','RVQ\rvq_projection_64to32.v','RVQ\rvq_core.v',
    'rtl\soundstream_encoder_rvq_top.v','tb\tb_soundstream_real_model.v'
)

New-Item -ItemType Directory -Force -Path (Join-Path $root 'logs') | Out-Null

Push-Location $root
try {
    if (-not (Test-Path -LiteralPath $work)) {
        & "$bin\vlib.exe" $work
        if ($LASTEXITCODE -ne 0) { throw 'vlib failed' }
    }

    & "$bin\vlog.exe" -sv -work $work -l $sramLog `
        third_party\opentitan_sram\prim_ram_1p_pkg.sv `
        third_party\opentitan_sram\prim_ram_2p_pkg.sv `
        third_party\opentitan_sram\prim_ram_1r1w_pkg.sv `
        third_party\opentitan_sram\prim_ram_1p.sv `
        third_party\opentitan_sram\prim_ram_1r1w.sv `
        rtl\opentitan_sram_1p_adapter.sv rtl\opentitan_sram_1r1w_adapter.sv
    if ($LASTEXITCODE -ne 0) { throw 'OpenTitan SRAM compile failed' }

    & "$bin\vlog.exe" -work $work -vlog01compat +define+REAL_MODEL_TEST `
        +incdir+rtl/flash +incdir+tb -l $compileLog $sources
    if ($LASTEXITCODE -ne 0) { throw 'Real-model Encoder/RVQ compile failed' }

    if ($CompileOnly) {
        Write-Host "Compile PASS: $(Join-Path $root $compileLog)"
        return
    }

    $missing = @($requiredData | Where-Object { -not (Test-Path -LiteralPath $_) })
    if ($missing.Count -ne 0) {
        $missingText = $missing -join [Environment]::NewLine
        throw "Real-model data package is incomplete:`n$missingText"
    }

    & "$bin\vsim.exe" -c -lib $work tb_soundstream_real_model `
        -l $simLog -do 'run -all; quit -f'
    if ($LASTEXITCODE -ne 0) { throw "Simulation failed; inspect $simLog" }
    Write-Host "Simulation log: $(Join-Path $root $simLog)"
} finally {
    Pop-Location
}
