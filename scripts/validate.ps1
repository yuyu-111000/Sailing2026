param(
    [string]$Cxx = "g++"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$build = Join-Path $root "build"
New-Item -ItemType Directory -Force -Path $build | Out-Null

$commonInclude = Join-Path $root "common/include"
$commonSource = Join-Path $root "common/src/infrared_link.cpp"
$f407Include = Join-Path $root "F407/include"
$l431Include = Join-Path $root "L431/include"

$f407Binary = Join-Path $build "f407_tests.exe"
& $Cxx -std=c++17 -Wall -Wextra -Wpedantic -Werror `
    "-I$commonInclude" "-I$f407Include" `
    $commonSource `
    (Join-Path $root "F407/src/infrared_publisher.cpp") `
    (Join-Path $root "F407/tests/test_main.cpp") `
    -o $f407Binary
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $f407Binary
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$l431Binary = Join-Path $build "l431_tests.exe"
& $Cxx -std=c++17 -Wall -Wextra -Wpedantic -Werror `
    "-I$commonInclude" "-I$l431Include" `
    $commonSource `
    (Join-Path $root "L431/src/jy61.cpp") `
    (Join-Path $root "L431/src/navigation.cpp") `
    (Join-Path $root "L431/src/controller.cpp") `
    (Join-Path $root "L431/tests/test_main.cpp") `
    -o $l431Binary
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $l431Binary
exit $LASTEXITCODE
