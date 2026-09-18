# Windows 版现场联动测试脚本（等价于 run_field_test.sh）。
# 功能：读取 test_case.yaml（拍照TCP + LabelMe标注）→ 调 POST /get_tcp_pose
#       解算目标TCP；work_jaka=false 时只输出坐标，不执行工作位运动。
# 前置条件：
#   1. 视觉服务已在后台运行（start_vision_service.ps1）；
#   2. 已用 run_field_capture.ps1 采集快照并记录拍照TCP；
#   3. LabelMe 标注路径已填入 test_case.yaml 的 case.labelme_file，
#      且 case.labelme_target_label 为目标标签。
# 用法: powershell -ExecutionPolicy Bypass -File run_field_test.ps1
# 默认从field_test/test_case.yaml读取live、base_label和code。
#   --live/--no-live 临时覆盖YAML中的实时拍照开关
#   --base-label N 临时覆盖YAML中的安装面板标签；传0可关闭
#   --code CODE    use_yolo=false时指定本次工件，例如9-8-1
#   --approach-mm N 临时覆盖法向预备距离；方向由当次柜体平面自动确定
#   --stay-at-work 到达工作位后停止，不执行退出或返回动作
param(
    [string]$Python = "D:/anaconda3/envs/ur_odcam/python.exe"
)
$ErrorActionPreference = "Stop"
$ProjectDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $ProjectDir

Write-Host "[1/1] 执行联动测试 (02_run_labelme_test.py) ..."
& $Python "field_test/02_run_labelme_test.py" @args
$code = $LASTEXITCODE
if ($code -ne 0) {
    Write-Host "[错误] 测试失败，退出码=$code"
    exit $code
}
Write-Host "[完成] 测试完成，结果见 field_test/output/test_result.json"
