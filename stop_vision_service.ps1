# 停止视觉检测服务（Windows PowerShell 版，等价于 stop_vision_service.sh）。
# 用法: powershell -ExecutionPolicy Bypass -File stop_vision_service.ps1
param([int]$Port = 48051)
$ErrorActionPreference = "Continue"

$procs = Get-CimInstance Win32_Process -Filter "Name='python.exe'" -ErrorAction SilentlyContinue |
    Where-Object { $_.CommandLine -and $_.CommandLine -like "*run_service.py*" }
if (-not $procs) {
    Write-Host "[提示] 视觉服务未在运行。"
} else {
    foreach ($p in $procs) {
        Write-Host "停止 PID=$($p.ProcessId) ..."
        Stop-Process -Id $p.ProcessId -Force -ErrorAction SilentlyContinue
    }
    Start-Sleep -Seconds 2
}

if (Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue) {
    Write-Host "[错误] 端口 $Port 仍被占用:"
    Get-NetTCPConnection -LocalPort $Port -State Listen -ErrorAction SilentlyContinue |
        Format-Table -AutoSize
} else {
    Write-Host "[完成] 服务已停止，端口 $Port 已释放。"
    Write-Host "提示: Windows 强杀进程可能残留相机驱动占用；若下次启动连不上相机，"
    Write-Host "      请给相机断电约10秒再上电。"
}
