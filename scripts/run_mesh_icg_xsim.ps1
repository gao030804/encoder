param(
    [ValidateRange(1,10)] [int] $Frames=10,
    [switch] $Integration,
    [switch] $Baseline,
    [ValidateSet('Realtime','Offline')] [string] $Mode='Realtime',
    [string] $VivadoBin='E:\Xilinx\Vivado\2018.3\bin'
)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$env:PROCESSOR_ARCHITECTURE='AMD64'
$tempDir=Join-Path $root 'build/xsim_temp'
if(-not(Test-Path -LiteralPath $tempDir)){New-Item -ItemType Directory -Path $tempDir | Out-Null}
$env:TEMP=$tempDir
$env:TMP=$tempDir
Push-Location $root
try {
    $definitions=@('-d','FPGA_TARGET','-d','SIM_MESH_ICG')
    $suffix='unit'
    $top='tb_mesh_clock_gate'
    $sources=@('rtl/clock/mesh_icg.v','rtl/PE.v','rtl/Systolic_Array.v',
        'rtl/input_skew_4lane.v','rtl/output_deskew_8lane.v',
        'rtl/mesh_core_4x8_systolic.v','rtl/mesh_adapter.v','tb/tb_mesh_clock_gate.v')
    $genericArgs=@()
    if($Integration){
        $suffix='encoder_'+$Mode.ToLower()
        if($Baseline){$suffix+='_baseline';$definitions+=@('-d','MESH_ICG_BYPASS')}
        $sources=@(Get-Content scripts/fpga_system_files.f | Where-Object {$_ -match '\.(v|sv)$'})
        $sources+=@('rtl/flash/P25Q32H.v','tb/tb_fpga_system_uart.v','tb/tb_mesh_icg_encoder_regression.v')
        $top='tb_mesh_icg_encoder_regression'
        $realtime=0
        if($Mode -eq 'Realtime'){$realtime=1}
        $genericArgs=@('--generic_top',('"FRAME_COUNT='+$Frames+'"'),
            '--generic_top',('"REALTIME_MODE='+$realtime+'"'))
    }elseif($Baseline){throw '-Baseline requires -Integration'}
    $snapshot='mesh_icg_'+$suffix
    $simLog='logs/'+$snapshot+'.log'
    & "$VivadoBin/xvlog.bat" -sv @definitions -i rtl/flash --log "logs/$snapshot`_compile.log" @sources
    if($LASTEXITCODE -ne 0){throw 'Mesh ICG compile failed'}
    & "$VivadoBin/xelab.bat" $top -s $snapshot --mt off @genericArgs --log "logs/$snapshot`_elaborate.log"
    if($LASTEXITCODE -ne 0){throw 'Mesh ICG elaboration failed'}
    & "$VivadoBin/xsim.bat" $snapshot --runall --log $simLog
    if($LASTEXITCODE -ne 0){throw 'Mesh ICG simulation failed'}
    $log=[IO.File]::ReadAllText((Join-Path $root $simLog))
    if($log -notmatch '\[TB_PASS\]' -or $log -match '\[TB_FAIL\]|\[ERROR\]|ERROR:|FATAL:'){
        throw "Mesh ICG checks failed: $simLog"
    }
    if($Integration){
        if($log -notmatch '\[MESH_ICG_CHECK_PASS\]' -or
            [regex]::Matches($log,'\[UART_FRAME_PASS\]').Count -ne $Frames -or
            [regex]::Matches($log,'\[LAYER_CHECK\]').Count -ne 63*$Frames -or
            [regex]::Matches($log,'\[RVQ_STAGE_CHECK\]').Count -ne 9*$Frames){
            throw "Mesh ICG integration coverage failed: $simLog"
        }
        if($Frames -eq 10){
            $modeName=$Mode.ToLower()
            & python scripts/decode_rvq_records.py --records "logs/fpga_uart_records_$modeName.hex" `
                --expect-golden tb/data/encoder_rvq_multiframe/output_all.hex `
                --output "logs/mesh_icg_software_$modeName.json"
            if($LASTEXITCODE -ne 0){throw 'PC reconstruction check failed'}
        }
    }
    Write-Host "Mesh ICG PASS: $simLog"
} finally {Pop-Location}
