param(
    [ValidateSet('All','Units','Full')] [string] $Phase='All',
    [ValidateSet(30,50)] [int] $ClockMHz=50,
    [ValidateSet(1,10)] [int] $Frames=10,
    [ValidateSet('Xsim','ModelSim')] [string] $Simulator='Xsim',
    [string] $EnRoot='E:\111\en',
    [string] $VivadoBin='E:\Xilinx\Vivado\2018.3\bin',
    [string] $ModelSimBin='E:\modelsim_24\win64pe',
    [ValidatePattern('^[A-Za-z0-9_]+$')] [string] $LogTag='qor_p0',
    [switch] $CompileOnly,
    [switch] $UseTechClockGate
)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$relativeLog="logs/${LogTag}_${ClockMHz}mhz"
$outDir=Join-Path $root $relativeLog
$tempDir=Join-Path $root 'build/qor_p0_temp'
foreach($dir in @($outDir,$tempDir)){
    if(-not(Test-Path -LiteralPath $dir)){New-Item -ItemType Directory -Path $dir | Out-Null}
}
$env:PROCESSOR_ARCHITECTURE='AMD64'
$env:TEMP=$tempDir
$env:TMP=$tempDir
Push-Location $root
try {
    # Read the shared ASIC bundle; use local P0 RTL and all five scan-port fixes.
    # Do not silently compile stale en copies with implicit pull-down input ports.
    $overrides=@{
        'conv1d_controller.v'='rtl/conv1d_controller.v'
        'rvq_backend_w8a16.v'='RVQ/rvq_backend_w8a16.v'
        'conv1d_engine_top.v'='rtl/conv1d_engine_top.v'
        'conv1d_engine_flash_top.v'='rtl/conv1d_engine_flash_top.v'
        'soundstream_encoder_top.v'='rtl/soundstream_encoder_top.v'
        'soundstream_encoder_rvq_w8a16_top.v'='rtl/soundstream_encoder_rvq_w8a16_top.v'
        'postprocess_8lane.v'='rtl/postprocess_8lane.v'
        'depthwise_conv1d_controller.v'='rtl/depthwise_conv1d_controller.v'
    }
    # Optional server-compatible gate; the default behavioral latch is unchanged.
    if($UseTechClockGate){$overrides['mesh_icg_tech.v']='rtl/clock/mesh_icg_tech.v'}
    $sources=@('third_party/opentitan_sram/prim_ram_1p_pkg.sv',
        'third_party/opentitan_sram/prim_ram_2p_pkg.sv',
        'third_party/opentitan_sram/prim_ram_1r1w_pkg.sv',
        'third_party/opentitan_sram/prim_ram_1p.sv',
        'third_party/opentitan_sram/prim_ram_1r1w.sv',
        'rtl/opentitan_sram_1p_adapter.sv','rtl/opentitan_sram_1r1w_adapter.sv')
    $overridden=@{}
    foreach($folder in @('adress','clock_gate','flash','input','lane','rvq_rtl','sa','system top')){
        foreach($file in Get-ChildItem -LiteralPath (Join-Path $EnRoot $folder) -File){
            if($file.Extension -eq '.v' -and $file.Name -ne 'P25Q32H.v'){
                if($overrides.ContainsKey($file.Name)){
                    $sources+=$overrides[$file.Name];$overridden[$file.Name]=$true
                }else{$sources+=$file.FullName}
            }
        }
    }
    foreach($name in $overrides.Keys){if(-not $overridden.ContainsKey($name)){throw "Missing RTL override target: $name"}}
    $sources+=@('rtl/flash_boot_parameter_store.v','rtl/flash/P25Q32H.v',
        'tb/tb_flash_encoder_rvq_full_frame.v','tb/tb_qor_p0_full_frame.v',
        'tb/tb_qor_p0_projection.v','tb/tb_qor_p0_post_stream.v',
        'tb/tb_mesh_scan_port_compat.v','tb/tb_mesh_icg_tech.v',
        'tb/tb_postprocess_pipeline.v','tb/tb_postprocess_depthwise.v',
        'tb/tb_postprocess_guard_sticky.v')
    foreach($p in $sources){if(-not(Test-Path -LiteralPath $p -PathType Leaf)){throw "Missing source: $p"}}
    foreach($p in $sources){
        if([IO.File]::ReadAllText((Resolve-Path -LiteralPath $p).Path) -match '(?m)^\s*input\s+tri0\s+mesh_scan_test_en\b'){
            throw "Unsupported scan input declaration reintroduced: $p"
        }
    }
    $sources | ForEach-Object {Get-FileHash -Algorithm SHA256 -LiteralPath $_} |
        Select-Object Path,Hash | Export-Csv -NoTypeInformation -Encoding UTF8 (Join-Path $outDir 'source_hashes.csv')
    if($Phase -ne 'Units'){
        $manifest=Get-Content tb/data/encoder_rvq_multiframe/manifest.json -Raw -Encoding UTF8 | ConvertFrom-Json
        if($Frames -eq 10 -and $manifest.source.mode -ne 'raw_pcm_independent_frames'){
            throw 'Full regression requires real independent PCM frames'
        }
    }
    if($Simulator -eq 'Xsim'){
        $compileDefines=@()
        if($UseTechClockGate){$compileDefines=@('-d','MESH_ICG_USE_TECH')}
        & "$VivadoBin/xvlog.bat" -sv -i . -i rtl/flash @compileDefines --log "$outDir/compile_xsim.log" @sources
        if($LASTEXITCODE -ne 0){throw 'P0 compilation failed'}
    }else{
        $work='build/qor_p0_work'
        if(-not(Test-Path -LiteralPath $work)){
            & "$ModelSimBin/vlib.exe" $work
            if($LASTEXITCODE -ne 0){throw 'vlib failed'}
        }
        $compileDefines=@()
        if($UseTechClockGate){$compileDefines=@('+define+MESH_ICG_USE_TECH')}
        & "$ModelSimBin/vlog.exe" -sv -work $work '+incdir+.' '+incdir+rtl/flash' @compileDefines -l "$outDir/compile_modelsim.log" @sources
        if($LASTEXITCODE -ne 0){throw 'P0 ModelSim compilation failed'}
    }
    $tops=@()
    if($Phase -ne 'Full'){
        if($UseTechClockGate){$tops+='tb_mesh_icg_tech'}else{$tops+='tb_mesh_scan_port_compat'}
        $tops+=@('tb_postprocess_guard_sticky','tb_postprocess_pipeline','tb_postprocess_depthwise',
            'tb_qor_p0_projection','tb_qor_p0_post_stream')
    }
    if($Phase -ne 'Units'){$tops+='tb_qor_p0_full_frame'}
    foreach($top in $tops){
        $simLog=Join-Path $outDir ($top+'.log')
        $generics=@()
        if($top -eq 'tb_qor_p0_full_frame'){
            if($Simulator -eq 'Xsim'){
                $generics=@('--generic_top',('"FRAME_COUNT='+$Frames+'"'),
                    '--generic_top',('"CLOCK_HZ='+($ClockMHz*1000000)+'"'),
                    '--generic_top',('"LOG_PREFIX='+$relativeLog+'/"'))
            }else{
                $generics=@("-gFRAME_COUNT=$Frames",('-gCLOCK_HZ='+($ClockMHz*1000000)),('-gLOG_PREFIX='+$relativeLog+'/'))
            }
        }
        if($Simulator -eq 'Xsim'){
            $snapshot=$top+"_${ClockMHz}mhz"
            if($UseTechClockGate){$snapshot+='_tech'}
            & "$VivadoBin/xelab.bat" $top -s $snapshot --mt off --timescale 1ns/1ps @generics --log "$outDir/${top}_elaborate.log"
            if($LASTEXITCODE -ne 0){throw "Elaboration failed: $top"}
            if($CompileOnly){continue}
            & "$VivadoBin/xsim.bat" $snapshot --runall --log $simLog
        }else{
            if($CompileOnly){continue}
            & "$ModelSimBin/vsim.exe" -c -lib $work $top @generics -l $simLog -do 'run -all; quit -f'
        }
        if($LASTEXITCODE -ne 0){throw "Simulation failed: $top"}
        $log=[IO.File]::ReadAllText($simLog)
        if($log -notmatch '\[TB_PASS\]' -or $log -match '\[\w*FAIL\]|\[ERROR\]|\*\* (Error|Fatal):|ERROR:|FATAL:'){
            throw "Dynamic checks failed: $simLog"
        }
        if($top -eq 'tb_qor_p0_full_frame'){
            foreach($item in @(@('LAYER_PASS',(63*$Frames)),@('RVQ_STAGE_PASS',(9*$Frames)),
                @('LATENT_PASS',$Frames),@('QOR_DEADLINE',$Frames))){
                if([regex]::Matches($log,('\['+$item[0]+'\]')).Count -ne $item[1]){
                    throw "Incomplete full-frame coverage: $($item[0])"
                }
            }
            if($log -notmatch '\[QOR_P0_PASS\]'){throw 'Missing optimized pipeline/deadline coverage'}
            if($Frames -eq 10){
                foreach($tag in @('FRAME_PASS','DMA_PASS')){
                    if([regex]::Matches($log,('\['+$tag+'\]')).Count -ne 10){throw "Missing $tag"}
                }
                & python scripts/decode_rvq_records.py --records "$outDir/encoded_records_multiframe.hex" `
                    --rvq-dir "$EnRoot/rvq" --expect-golden tb/data/encoder_rvq_multiframe/output_all.hex `
                    --output "$outDir/software_reconstruction.json"
                if($LASTEXITCODE -ne 0){throw 'Actual DMA record reconstruction failed'}
            }
        }
        Write-Host "Dynamic PASS: $simLog"
    }
    if($CompileOnly){Write-Host 'Compile/elaborate checks only; no dynamic PASS claimed'}
} finally {Pop-Location}
