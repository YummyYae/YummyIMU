param(
    [string] $ProjectRoot = (Resolve-Path "$PSScriptRoot\..").Path,
    [string] $KeilRoot = "D:\Programing\Keil5",
    [string] $Adb = "adb",
    [string] $LocalMcpPort = "18080",
    [string] $LocalOpenOcdPort = "14444",
    [switch] $NoBuild,
    [switch] $NoFlash,
    [switch] $SkipMcpConnect
)

$ErrorActionPreference = "Stop"

function Invoke-Checked {
    param(
        [Parameter(Mandatory = $true)] [string] $FilePath,
        [Parameter(Mandatory = $true)] [string[]] $Arguments
    )

    & $FilePath @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "$FilePath failed with exit code $LASTEXITCODE"
    }
}

function Read-OpenOcd {
    param(
        [Parameter(Mandatory = $true)] $Stream,
        [Parameter(Mandatory = $true)] $Reader,
        [int] $TimeoutMs = 1000
    )

    $deadline = (Get-Date).AddMilliseconds($TimeoutMs)
    $text = ""
    $buffer = New-Object char[] 8192

    while ((Get-Date) -lt $deadline) {
        while ($Stream.DataAvailable) {
            $count = $Reader.Read($buffer, 0, $buffer.Length)
            if ($count -gt 0) {
                $text += -join $buffer[0..($count - 1)]
            }
        }

        if ($text -match ">\s*$") {
            break
        }

        Start-Sleep -Milliseconds 100
    }

    return $text
}

function Invoke-OpenOcdCommand {
    param(
        [Parameter(Mandatory = $true)] $Stream,
        [Parameter(Mandatory = $true)] $Writer,
        [Parameter(Mandatory = $true)] $Reader,
        [Parameter(Mandatory = $true)] [string] $Command,
        [int] $TimeoutMs = 3000
    )

    Write-Host "openocd> $Command"
    $Writer.WriteLine($Command)
    $response = Read-OpenOcd -Stream $Stream -Reader $Reader -TimeoutMs $TimeoutMs
    Write-Host $response
    return $response
}

function Invoke-McpTool {
    param(
        [Parameter(Mandatory = $true)] [string] $Name,
        [hashtable] $Arguments = @{}
    )

    $body = @{
        jsonrpc = "2.0"
        id = [int](Get-Random -Minimum 1 -Maximum 2147483647)
        method = "tools/call"
        params = @{
            name = $Name
            arguments = $Arguments
        }
    } | ConvertTo-Json -Depth 20 -Compress

    return Invoke-RestMethod -Uri "http://127.0.0.1:$LocalMcpPort/mcp" -Method Post -ContentType "application/json" -Body $body -TimeoutSec 30
}

$uv4 = Join-Path $KeilRoot "UV4\UV4.exe"
$fromelf = Join-Path $KeilRoot "ARM\ARMCLANG\bin\fromelf.exe"
$project = Join-Path $ProjectRoot "keil\ncontroller.uvprojx"
$buildLog = Join-Path $ProjectRoot "keil\build_codex.log"
$axf = Join-Path $ProjectRoot "keil\Objects\ncontroller.axf"
$bin = Join-Path $ProjectRoot "keil\Objects\ncontroller.bin"
$remoteBin = "/tmp/ncontroller.bin"

if (-not $NoBuild) {
    Write-Host "Building Keil project..."
    Invoke-Checked -FilePath $uv4 -Arguments @("-r", $project, "-j0", "-o", $buildLog)
    Get-Content -LiteralPath $buildLog | Select-Object -Last 20

    Write-Host "Refreshing binary with fromelf..."
    Invoke-Checked -FilePath $fromelf -Arguments @("--bin", "--output", $bin, $axf)
}

$binItem = Get-Item -LiteralPath $bin
$binHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $bin).Hash
$binBytes = [IO.File]::ReadAllBytes($bin)
$binText = [Text.Encoding]::ASCII.GetString($binBytes)
$uartFormat = "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f"
$uartFormatOffset = $binText.IndexOf($uartFormat)
Write-Host "Firmware: $($binItem.FullName)"
Write-Host "Size: $($binItem.Length) bytes"
Write-Host "SHA256: $binHash"
if ($uartFormatOffset -ge 0) {
    Write-Host ("9-field UART format offset: 0x{0:x}" -f $uartFormatOffset)
}

if ($NoFlash) {
    return
}

Write-Host "Checking ADB device..."
Invoke-Checked -FilePath $Adb -Arguments @("get-state")

if (-not $SkipMcpConnect) {
    Write-Host "Forwarding AIxProbe MCP port through ADB..."
    Invoke-Checked -FilePath $Adb -Arguments @("forward", "tcp:$LocalMcpPort", "tcp:8080")

    Write-Host "Starting OpenOCD through AIxProbe MCP..."
    $connect = Invoke-McpTool -Name "connect"
    $connectText = ($connect.result.content | ForEach-Object { $_.text }) -join "`n"
    Write-Host $connectText

    Start-Sleep -Milliseconds 500
    $halt = Invoke-McpTool -Name "halt"
    $haltText = ($halt.result.content | ForEach-Object { $_.text }) -join "`n"
    Write-Host $haltText
    if ($halt.result.isError -or $haltText -match "cannot read IDR|OpenOCD process died|no active session") {
        throw "AIxProbe/OpenOCD could not connect to target SWD. Check target power, GND, SWDIO, SWCLK, and reset/boot state."
    }
}

Write-Host "Forwarding OpenOCD telnet port through ADB..."
Invoke-Checked -FilePath $Adb -Arguments @("forward", "tcp:$LocalOpenOcdPort", "tcp:4444")

Write-Host "Uploading firmware to debug box..."
Invoke-Checked -FilePath $Adb -Arguments @("push", $bin, $remoteBin)

$client = New-Object Net.Sockets.TcpClient("127.0.0.1", [int] $LocalOpenOcdPort)
try {
    $stream = $client.GetStream()
    $stream.ReadTimeout = 5000
    $writer = New-Object IO.StreamWriter($stream, [Text.Encoding]::ASCII)
    $writer.AutoFlush = $true
    $reader = New-Object IO.StreamReader($stream, [Text.Encoding]::ASCII)

    Read-OpenOcd -Stream $stream -Reader $reader -TimeoutMs 1000 | Write-Host
    Invoke-OpenOcdCommand -Stream $stream -Writer $writer -Reader $reader -Command "targets" -TimeoutMs 2000 | Out-Null
    Invoke-OpenOcdCommand -Stream $stream -Writer $writer -Reader $reader -Command "reset halt" -TimeoutMs 5000 | Out-Null
    $flashResponse = Invoke-OpenOcdCommand -Stream $stream -Writer $writer -Reader $reader -Command "flash write_image erase $remoteBin 0x0 bin" -TimeoutMs 70000
    if ($flashResponse -match "(?i)(failed|error writing|can't|cannot)") {
        throw "OpenOCD flash write failed."
    }
    if ($flashResponse -notmatch "wrote\s+\d+\s+bytes") {
        Write-Warning "OpenOCD did not print the usual byte-count success line; continuing with readback checks."
    }

    $vector = Invoke-OpenOcdCommand -Stream $stream -Writer $writer -Reader $reader -Command "mdw 0x0 4" -TimeoutMs 3000
    if ($vector -notmatch "2020[0-9a-fA-F]{4}") {
        throw "Vector table readback did not look like MSPM0 SRAM stack pointer."
    }

    if ($uartFormatOffset -ge 0) {
        $probeAddress = "0x{0:x}" -f $uartFormatOffset
        $formatProbe = Invoke-OpenOcdCommand -Stream $stream -Writer $writer -Reader $reader -Command "mdb $probeAddress 64" -TimeoutMs 3000
        if ($formatProbe -notmatch "25 2e 33 66") {
            throw "Did not see the UART float format marker at $probeAddress after flashing."
        }
    }
    else {
        Write-Warning "Could not find the 9-field UART format string in the local binary."
    }

    Invoke-OpenOcdCommand -Stream $stream -Writer $writer -Reader $reader -Command "reset run" -TimeoutMs 3000 | Out-Null
}
finally {
    if ($client.Connected) {
        $client.Close()
    }
}

Write-Host "ADB wired flash complete."
