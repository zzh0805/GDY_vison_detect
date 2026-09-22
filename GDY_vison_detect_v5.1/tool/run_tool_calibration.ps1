param(
    [string]$Python = "python",
    [string]$Config = ""
)

$ToolDir = Split-Path -Parent $MyInvocation.MyCommand.Path
if (-not $Config) {
    $Config = Join-Path $ToolDir "tool_calibration.yaml"
}

& $Python (Join-Path $ToolDir "tool_calibration_workflow.py") --config $Config
exit $LASTEXITCODE
