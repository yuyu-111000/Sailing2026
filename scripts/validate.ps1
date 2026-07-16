param(
    [string]$Cxx = "g++"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$build = Join-Path $root "build"
New-Item -ItemType Directory -Force -Path $build | Out-Null
$binary = Join-Path $build "lower_computer_tests.exe"

& $Cxx -std=c++17 -Wall -Wextra -Wpedantic -Werror `
    -I(Join-Path $root "lower_computer/include") `
    (Join-Path $root "lower_computer/src/sensors.cpp") `
    (Join-Path $root "lower_computer/src/navigation.cpp") `
    (Join-Path $root "lower_computer/src/controller.cpp") `
    (Join-Path $root "lower_computer/tests/test_main.cpp") `
    -o $binary
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $binary
exit $LASTEXITCODE
