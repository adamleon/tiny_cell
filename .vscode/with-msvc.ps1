[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet('configure', 'build')]
    [string]$Action = 'build'
)

$ErrorActionPreference = 'Stop'

$cacheFile = Join-Path $PSScriptRoot '.vcvars-cache.json'

function Set-EnvFromVcvars {
    $vs = & 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe' -latest -property installationPath
    if (-not $vs) { throw 'No Visual Studio installation found via vswhere.' }
    $vcvars = Join-Path $vs 'VC\Auxiliary\Build\vcvars64.bat'
    if (-not (Test-Path $vcvars)) { throw "vcvars64.bat not found at $vcvars" }

    cmd /c "`"$vcvars`" >nul && set" | ForEach-Object {
        if ($_ -match '^([^=]+)=(.*)$') {
            Set-Item -Path "Env:$($matches[1])" -Value $matches[2]
        }
    }

    $envSnapshot = @{}
    Get-ChildItem Env: | ForEach-Object { $envSnapshot[$_.Name] = $_.Value }
    @{ vcvarsPath = $vcvars; env = $envSnapshot } |
        ConvertTo-Json -Depth 3 | Set-Content $cacheFile
}

$cacheHit = $false
if (Test-Path $cacheFile) {
    try {
        $cached = Get-Content $cacheFile -Raw | ConvertFrom-Json
        if ($cached.vcvarsPath -and (Test-Path $cached.vcvarsPath)) {
            $cacheTime  = (Get-Item $cacheFile).LastWriteTime
            $vcvarsTime = (Get-Item $cached.vcvarsPath).LastWriteTime
            if ($cacheTime -gt $vcvarsTime) {
                $cached.env.PSObject.Properties | ForEach-Object {
                    Set-Item -Path "Env:$($_.Name)" -Value $_.Value
                }
                $cacheHit = $true
            }
        }
    } catch {
        # corrupt cache; fall through to fresh vcvars
    }
}

if (-not $cacheHit) {
    Set-EnvFromVcvars
}

$workspace = Split-Path -Parent $PSScriptRoot
$buildDir  = Join-Path $workspace 'build-msvc'

$cmakeArgs = @(
    '-S', $workspace,
    '-B', $buildDir,
    '-G', 'Ninja',
    '-DCMAKE_BUILD_TYPE=Release',
    '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'
)

switch ($Action) {
    'configure' {
        cmake @cmakeArgs
    }
    'build' {
        if (-not (Test-Path (Join-Path $buildDir 'CMakeCache.txt'))) {
            cmake @cmakeArgs
            if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
        }
        cmake --build $buildDir --parallel
    }
}

exit $LASTEXITCODE
