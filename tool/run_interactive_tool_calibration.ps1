param(
    [string]$Python = "python",
    [string]$Config = ""
)

$ToolDir = Split-Path -Parent $MyInvocation.MyCommand.Path
if (-not $Config) {
    $Config = Join-Path $ToolDir "interactive_tool_calibration.yaml"
}

& $Python (Join-Path $ToolDir "interactive_tool_calibration.py") --config $Config
exit $LASTEXITCODE
