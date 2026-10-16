param([switch] $CompileOnly)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$bin = 'E:\modelsim_24\win64pe'
$work = Join-Path $root 'build\flash_encoder_frame_work'
Push-Location $root
try {
    & python scripts\prepare_w8a16_encoder_test_data.py
    if ($LASTEXITCODE -ne 0) { throw 'W8A16 data conversion failed' }
    & python scripts/prepare_flash_encoder_frame.py
    if ($LASTEXITCODE -ne 0) { throw 'Flash frame image generation failed' }
    if (-not (Test-Path -LiteralPath $work)) { & "$bin\vlib.exe" $work }
    & "$bin\vlog.exe" -sv -work $work -l logs\flash_encoder_frame_sram_compile.log `
        third_party\opentitan_sram\prim_ram_1p_pkg.sv `
        third_party\opentitan_sram\prim_ram_2p_pkg.sv `
        third_party\opentitan_sram\prim_ram_1r1w_pkg.sv `
        third_party\opentitan_sram\prim_ram_1p.sv `
        third_party\opentitan_sram\prim_ram_1r1w.sv `
        rtl\opentitan_sram_1p_adapter.sv rtl\opentitan_sram_1r1w_adapter.sv
    if ($LASTEXITCODE -ne 0) { throw 'SRAM compile failed' }
    $src = @(
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
        'rtl\pcm16_input_frontend.v','rtl\flash\P25Q32H.v','rtl\flash_boot_parameter_store.v','tb\tb_flash_encoder_full_frame.v'
    )
    & "$bin\vlog.exe" -work $work -vlog01compat +define+W8A16_GOLDEN_TEST `
        +incdir+rtl/flash -l logs\flash_encoder_frame_compile.log $src
    if ($LASTEXITCODE -ne 0) { throw 'W8A16 Encoder compile failed' }
    if ($CompileOnly) { Write-Host 'W8A16 compile PASS'; return }
    & "$bin\vsim.exe" -c -lib $work tb_flash_encoder_full_frame `
        -l logs\flash_encoder_frame.log -do 'run -all; quit -f'
    if ($LASTEXITCODE -ne 0) { throw 'W8A16 simulation failed' }
    $pass = Select-String -LiteralPath 'logs/flash_encoder_frame.log' -SimpleMatch '[TB_PASS]' -Quiet
    $fail = Select-String -LiteralPath 'logs/flash_encoder_frame.log' -Pattern '\[TB_FAIL\]|\[LAYER_FAIL\]|\[ERROR\]|\*\* (Error|Fatal):' -Quiet
    $layers = @(Select-String -LiteralPath 'logs/flash_encoder_frame.log' -SimpleMatch '[LAYER_PASS]').Count
    $latentPass = Select-String -LiteralPath 'logs/flash_encoder_frame.log' -SimpleMatch '[LATENT_PASS]' -Quiet
    if (-not $pass -or $fail -or $layers -ne 63 -or -not $latentPass) { throw 'Flash frame checks failed; inspect logs/flash_encoder_frame.log' }
    Write-Host 'Flash frame PASS: 63 layers, boot and SRAM transport verified'
} finally { Pop-Location }
