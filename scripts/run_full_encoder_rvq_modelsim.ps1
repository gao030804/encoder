# 编译并运行人工数据或真实.pt/.wav导出数据的完整帧回归。
param([switch] $RealModel)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$bin  = 'E:\modelsim_24\win64pe'
$work = Join-Path $root $(if ($RealModel) {
    'build\soundstream_real_model_work'
} else {
    'build\full_encoder_rvq_work'
})
$logs = Join-Path $root 'logs'
New-Item -ItemType Directory -Force -Path $logs | Out-Null

Push-Location $root
try {
    if ($RealModel) {
        if (-not (Test-Path -LiteralPath 'tb\data\real_audio')) {
            throw 'Missing tb\data\real_audio; export the real model/WAV package first'
        }
        Write-Host 'Mode: real .pt/.wav exported integer package'
    } else {
        & python scripts\generate_full_encoder_rvq_test_data.py
        if ($LASTEXITCODE -ne 0) { throw 'Golden data generation failed' }
        Write-Host 'Mode: deterministic artificial full-frame regression'
    }
    if (-not (Test-Path -LiteralPath $work)) {
        & "$bin\vlib.exe" $work
        if ($LASTEXITCODE -ne 0) { throw 'vlib failed' }
    }

    & "$bin\vlog.exe" -sv -work $work -l logs\full_encoder_rvq_sram_compile.log `
        third_party\opentitan_sram\prim_ram_1p_pkg.sv `
        third_party\opentitan_sram\prim_ram_2p_pkg.sv `
        third_party\opentitan_sram\prim_ram_1r1w_pkg.sv `
        third_party\opentitan_sram\prim_ram_1p.sv `
        third_party\opentitan_sram\prim_ram_1r1w.sv `
        rtl\opentitan_sram_1p_adapter.sv rtl\opentitan_sram_1r1w_adapter.sv
    if ($LASTEXITCODE -ne 0) { throw 'OpenTitan SRAM compile failed' }

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
        'rtl\soundstream_encoder_rvq_top.v'
    )
    if ($RealModel) {
        $sources += 'tb\tb_soundstream_real_model.v'
        & "$bin\vlog.exe" -work $work -vlog01compat +define+REAL_MODEL_TEST `
            +incdir+rtl/flash +incdir+tb -l logs\soundstream_real_model_compile.log $sources
    } else {
        $sources += 'tb\tb_soundstream_encoder_rvq_full_frame.v'
        & "$bin\vlog.exe" -work $work -vlog01compat +define+FULL_ENCODER_RVQ_TEST `
            +incdir+rtl/flash -l logs\full_encoder_rvq_compile.log $sources
    }
    if ($LASTEXITCODE -ne 0) { throw 'Encoder/RVQ RTL compile failed' }

    $top = if ($RealModel) {'tb_soundstream_real_model'} else {'tb_soundstream_encoder_rvq_full_frame'}
    $simLog = if ($RealModel) {'logs\soundstream_real_model.log'} else {'logs\full_encoder_rvq.log'}
    & "$bin\vsim.exe" -c -lib $work $top -l $simLog -do 'run -all; quit -f'
    if ($LASTEXITCODE -ne 0) { throw "Simulation failed; inspect $simLog" }
    Write-Host "Simulation log: $(Join-Path $root $simLog)"
} finally {
    Pop-Location
}
