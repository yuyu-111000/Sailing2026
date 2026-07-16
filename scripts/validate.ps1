param(
    [string]$Python = "python",
    [string]$Cxx = "g++"
)

$ErrorActionPreference = "Stop"
$root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path

Push-Location $root
try {
    & $Python -c "import sys; assert sys.version_info >= (3, 12), sys.version"
    if ($LASTEXITCODE -ne 0) { throw "Python 3.12+ check failed" }

    Push-Location "upper_computer"
    try {
        & $Python -m compileall -q sailing tests
        if ($LASTEXITCODE -ne 0) { throw "Python compile check failed" }
        & $Python -m unittest discover -s tests -v
        if ($LASTEXITCODE -ne 0) { throw "Python tests failed" }
    }
    finally {
        Pop-Location
    }

    New-Item -ItemType Directory -Force -Path "build" | Out-Null
    & $Cxx -std=c++17 -Wall -Wextra -Wpedantic -Werror `
        -Ilower_computer/include `
        lower_computer/src/protocol.cpp `
        lower_computer/src/safety_supervisor.cpp `
        lower_computer/tests/test_main.cpp `
        -o build/lower_computer_tests.exe
    if ($LASTEXITCODE -ne 0) { throw "C++17 build failed" }

    & ".\build\lower_computer_tests.exe" "shared\test-vectors\protocol-v1.tsv"
    if ($LASTEXITCODE -ne 0) { throw "C++ tests failed" }

    Write-Host "All host-only Sailing2026 checks passed."
}
finally {
    Pop-Location
}
