param(
    [string]$Python = "python"
)

$ErrorActionPreference = "Stop"
$upper = (Resolve-Path (Join-Path $PSScriptRoot "..\upper_computer")).Path

Push-Location $upper
try {
    & $Python -m examples.run_scenarios
    if ($LASTEXITCODE -ne 0) { throw "Host-only scenarios failed" }
}
finally {
    Pop-Location
}
