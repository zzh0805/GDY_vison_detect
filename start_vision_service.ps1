# 一键后台启动视觉检测服务（Windows PowerShell 版，等价于 start_vision_service.sh）。
# 用法: powershell -ExecutionPolicy Bypass -File start_vision_service.ps1
param(
    [string]$Python = "D:/anaconda3/envs/ur_odcam/python.exe",
    [int]$Port = 48051
)
$ErrorActionPreference = "Stop"
$ProjectDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $ProjectDir
$LogFile = Join-Path $ProjectDir "service.log"
$LogErr = Join-Path $ProjectDir "service.log.err"

# 已在运行则直接提示
$existing = Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue |
    Where-Object { $_.CommandLine -and $_.CommandLine -like "*run_service.py*" }
if ($existing) {
    Write-Host "[提示] 视觉服务已在运行: PID=$($existing.ProcessId)"
    exit 0
}

Write-Host "[1/3] 后台启动视觉服务..."
$proc = Start-Process -FilePath $Python `
    -ArgumentList @("run_service.py") `
    -WorkingDirectory $ProjectDir `
    -WindowStyle Hidden `
    -RedirectStandardOutput $LogFile `
    -RedirectStandardError $LogErr `
    -PassThru
Write-Host "     启动PID=$($proc.Id)，日志: $LogFile"

Write-Host "[2/3] 等待端口 $Port 就绪(最多60秒)..."
$ready = $false
for ($i = 1; $i -le 60; $i++) {
    if (Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue) {
        Write-Host "     端口 $Port 已就绪 (${i}秒)"
        $ready = $true
        break
    }
    if ($proc.HasExited) {
        Write-Host "[错误] 服务进程已退出，最近日志:"
        Get-Content $LogFile -Tail 20 -ErrorAction SilentlyContinue
        Get-Content $LogErr -Tail 20 -ErrorAction SilentlyContinue
        exit 1
    }
    Start-Sleep -Seconds 1
}
if (-not $ready) {
    Write-Host "[错误] 端口 $Port 未监听，最近日志:"
    Get-Content $LogFile -Tail 20 -ErrorAction SilentlyContinue
    Get-Content $LogErr -Tail 20 -ErrorAction SilentlyContinue
    exit 1
}
Write-Host "视觉服务已启动并监听 $Port。"
