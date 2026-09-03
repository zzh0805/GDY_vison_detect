# Windows 版现场采集脚本（等价于 run_field_capture.sh）。
# 功能：连接 JAKA 读取当前 TCP（只读、不运动）→ 调 POST /snapshot 拍照
#       → 把图像路径和拍照 TCP 写入 field_test/test_case.yaml。
# 前置条件：
#   1. 视觉服务已在后台运行（start_vision_service.ps1）；
#   2. JAKA 机器人网络可达（test_case.yaml 的 jaka_test.ip）；
#   3. JAKA Windows SDK 默认路径存在（jaka_adapter.py 已内置），
#      或通过 test_case.yaml 的 jaka_test.sdk_path 指定。
# 用法: powershell -ExecutionPolicy Bypass -File run_field_capture.ps1
param(
    [string]$Python = "D:/anaconda3/envs/ur_odcam/python.exe"
)
$ErrorActionPreference = "Stop"
$ProjectDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $ProjectDir

# 若 JAKA SDK 不在默认路径，可取消注释并改成实际路径：
# $env:JAKA_SDK_PATH = "D:\edge_downloade\20260104145805A007\SDK V2.3.1_beta3\Windows\WINDOWS_MSVC_V142_x64\Windows\python3\x64"

Write-Host "[1/1] 执行现场采集 (01_capture_test_case.py) ..."
& $Python "field_test/01_capture_test_case.py" @args
$code = $LASTEXITCODE
if ($code -ne 0) {
    Write-Host "[错误] 采集失败，退出码=$code"
    exit $code
}
Write-Host "[完成] 采集完成，test_case.yaml 已更新。"
