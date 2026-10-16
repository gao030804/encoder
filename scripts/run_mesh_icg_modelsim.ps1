param(
    [ValidateRange(1,10)] [int] $Frames=10,
    [switch] $Integration,
    [switch] $Baseline,
    [switch] $CompileOnly,
    [ValidateSet('Realtime','Offline')] [string] $Mode='Realtime',
    [string] $ModelSimBin='E:\modelsim_24\win64pe'
)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
Push-Location $root
try {
    $work='build/mesh_icg_modelsim_work'
    if(-not(Test-Path -LiteralPath $work)){
        & "$ModelSimBin/vlib.exe" $work
        if($LASTEXITCODE -ne 0){throw 'vlib failed'}
    }
    $defines=@('+define+FPGA_TARGET','+define+SIM_MESH_ICG')
    $suffix='unit';$top='tb_mesh_clock_gate';$genericArgs=@()
    $sources=@('rtl/clock/mesh_icg.v','rtl/PE.v','rtl/Systolic_Array.v',
        'rtl/input_skew_4lane.v','rtl/output_deskew_8lane.v',
        'rtl/mesh_core_4x8_systolic.v','rtl/mesh_adapter.v','tb/tb_mesh_clock_gate.v')
    if($Integration){
        $suffix='encoder_'+$Mode.ToLower()
        if($Baseline){$suffix+='_baseline';$defines+='+define+MESH_ICG_BYPASS'}
        $sources=@(Get-Content scripts/fpga_system_files.f | Where-Object {$_ -match '\.(v|sv)$'})
        $sources+=@('rtl/flash/P25Q32H.v','tb/tb_fpga_system_uart.v','tb/tb_mesh_icg_encoder_regression.v')
        $top='tb_mesh_icg_encoder_regression'
        $realtime=0;if($Mode -eq 'Realtime'){$realtime=1}
        $genericArgs=@("-gFRAME_COUNT=$Frames","-gREALTIME_MODE=$realtime")
    }elseif($Baseline){throw '-Baseline requires -Integration'}
    $simLog="logs/mesh_icg_modelsim_$suffix.log"
    & "$ModelSimBin/vlog.exe" -sv -work $work @defines '+incdir+rtl/flash' `
        -l "logs/mesh_icg_modelsim_$suffix`_compile.log" @sources
    if($LASTEXITCODE -ne 0){throw 'Mesh ICG ModelSim compile failed'}
    if($CompileOnly){Write-Host 'Compile PASS only; dynamic simulation not run';return}
    & "$ModelSimBin/vsim.exe" -c -lib $work $top @genericArgs -l $simLog -do 'run -all; quit -f'
    if($LASTEXITCODE -ne 0){throw 'Mesh ICG ModelSim simulation failed or license unavailable'}
    $log=[IO.File]::ReadAllText((Join-Path $root $simLog))
    if($log -notmatch '\[TB_PASS\]' -or $log -match '\[TB_FAIL\]|\[ERROR\]|\*\* (Error|Fatal):'){
        throw "Mesh ICG checks failed: $simLog"
    }
    if($Integration -and ($log -notmatch '\[MESH_ICG_CHECK_PASS\]' -or
        [regex]::Matches($log,'\[UART_FRAME_PASS\]').Count -ne $Frames -or
        [regex]::Matches($log,'\[LAYER_CHECK\]').Count -ne 63*$Frames -or
        [regex]::Matches($log,'\[RVQ_STAGE_CHECK\]').Count -ne 9*$Frames)){
        throw "Mesh ICG coverage failed: $simLog"
    }
    Write-Host "Mesh ICG ModelSim PASS: $simLog"
} finally {Pop-Location}
