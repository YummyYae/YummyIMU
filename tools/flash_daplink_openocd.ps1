param(
    [string] $ProjectRoot = (Resolve-Path "$PSScriptRoot\..").Path,
    [string] $KeilRoot = "D:\Programing\Keil5",
    [string] $OpenOcd = "D:\Programing\xpack-openocd-0.12.0-7\bin\openocd.exe",
    [string] $OpenOcdScripts = "D:\Programing\xpack-openocd-0.12.0-7\openocd\scripts",
    [int] $AdapterSpeedKhz = 100,
    [switch] $NoBuild,
    [switch] $NoFlash
)

$ErrorActionPreference = "Stop"

function Invoke-Checked {
    param(
        [Parameter(Mandatory = $true)] [string] $FilePath,
        [Parameter(Mandatory = $true)] [string[]] $Arguments
    )

    & $FilePath @Arguments
    if (($null -ne $LASTEXITCODE) -and ($LASTEXITCODE -ne 0)) {
        throw "$FilePath failed with exit code $LASTEXITCODE"
    }
}

$uv4 = Join-Path $KeilRoot "UV4\uVision.com"
$fromelf = Join-Path $KeilRoot "ARM\ARMCLANG\bin\fromelf.exe"
$project = Join-Path $ProjectRoot "keil\ncontroller.uvprojx"
$buildLog = Join-Path $ProjectRoot "keil\build_codex.log"
$axf = Join-Path $ProjectRoot "keil\Objects\ncontroller.axf"
$bin = Join-Path $ProjectRoot "keil\Objects\ncontroller.bin"

if (-not $NoBuild) {
    Write-Host "Building Keil project..."
    $buildStart = Get-Date
    if (Test-Path -LiteralPath $buildLog) {
        Remove-Item -LiteralPath $buildLog -Force
    }
    Invoke-Checked -FilePath $uv4 -Arguments @("-r", $project, "-j0", "-o", $buildLog)

    $deadline = (Get-Date).AddSeconds(60)
    do {
        Start-Sleep -Milliseconds 200
        $logText = if (Test-Path -LiteralPath $buildLog) {
            Get-Content -LiteralPath $buildLog -Raw -ErrorAction SilentlyContinue
        } else {
            ""
        }
        $logReady = (Test-Path -LiteralPath $buildLog) -and
            ((Get-Item -LiteralPath $buildLog).LastWriteTime -ge $buildStart.AddSeconds(-2)) -and
            ($logText -match 'ncontroller\.axf" - \d+ Error\(s\), \d+ Warning\(s\)\.')
        $axfReady = (Test-Path -LiteralPath $axf) -and
            ((Get-Item -LiteralPath $axf).LastWriteTime -ge $buildStart.AddSeconds(-2))
    } while ((-not ($logReady -and $axfReady)) -and ((Get-Date) -lt $deadline))

    if (-not ($logReady -and $axfReady)) {
        throw "Timed out waiting for Keil build outputs to refresh."
    }

    Get-Content -LiteralPath $buildLog | Select-Object -Last 20
    if ((Get-Content -LiteralPath $buildLog -Raw) -notmatch 'ncontroller\.axf" - 0 Error\(s\),') {
        throw "Keil build did not finish cleanly."
    }

    Write-Host "Refreshing binary with fromelf..."
    if (Test-Path -LiteralPath $bin) {
        Remove-Item -LiteralPath $bin -Force
    }
    Invoke-Checked -FilePath $fromelf -Arguments @("--bin", "--output", $bin, $axf)
    if (-not (Test-Path -LiteralPath $bin)) {
        throw "fromelf did not create $bin"
    }
}

$binItem = Get-Item -LiteralPath $bin
$binHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $bin).Hash
$binText = [Text.Encoding]::ASCII.GetString([IO.File]::ReadAllBytes($bin))
$formatOffset = $binText.IndexOf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f")

Write-Host "Firmware: $($binItem.FullName)"
Write-Host "Size: $($binItem.Length) bytes"
Write-Host "SHA256: $binHash"
if ($formatOffset -ge 0) {
    Write-Host ("9-field UART format offset: 0x{0:x}" -f $formatOffset)
}

if ($NoFlash) {
    return
}

$binForOpenOcd = $bin -replace "\\", "/"
$commands = @(
    "transport select swd",
    "adapter speed $AdapterSpeedKhz",
    "init",
    "reset halt",
    "flash write_image erase $binForOpenOcd 0x0 bin",
    "mdw 0x0 4"
)

if ($formatOffset -ge 0) {
    $commands += ("mdb 0x{0:x} 64" -f $formatOffset)
}

$commands += @("reset run", "shutdown")
$openOcdCommand = $commands -join "; "

Write-Host "Flashing through local DAPLink CMSIS-DAP..."
$oldErrorActionPreference = $ErrorActionPreference
$ErrorActionPreference = "Continue"
try {
    $openOcdOutput = & $OpenOcd `
        -s $OpenOcdScripts `
        -f interface/cmsis-dap.cfg `
        -f target/ti_mspm0.cfg `
        -c $openOcdCommand 2>&1
    $openOcdExitCode = $LASTEXITCODE
}
finally {
    $ErrorActionPreference = $oldErrorActionPreference
}

$openOcdOutput | ForEach-Object { Write-Host $_ }

if ($openOcdExitCode -ne 0) {
    throw "OpenOCD failed with exit code $openOcdExitCode"
}

Write-Host "DAPLink flash complete."
