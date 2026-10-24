param(
    [ValidateRange(0,4)][int]$Mode = 0,
    [ValidateRange(0.0,1.0)][double]$Probability = 0.1,
    [ValidateRange(0.001,86400.0)][double]$Seconds = 8,
    [int]$HoistSeed = 42,
    [ValidateRange(1,1000)][int]$OhtCount = 400,
    [ValidateRange(0.001,48.0)][double]$Hours = 1,
    [string]$Python = 'python'
)
$ErrorActionPreference = 'Stop'
$names = @('HOIST_MODE','HOIST_PROBABILITY','HOIST_SECONDS','HOIST_SEED','OHT_COUNT','SIMULATION_HOURS','RESULT_JSON_PATH','SPAWN_INTERVAL_SECONDS','PENDING_JOB_LIMIT','STOP_AFTER_SECONDS','STOP_WHEN_ALL_LOTS_DONE','APPLY_FIXES','CASE_DIRECTORY','ENABLE_RING_CONTROL','RING_CAPACITY_MARGIN')
$saved = @{}
foreach ($name in $names) { $saved[$name] = [Environment]::GetEnvironmentVariable($name,'Process') }
Push-Location $PSScriptRoot
try {
    $culture = [Globalization.CultureInfo]::InvariantCulture
    $env:HOIST_MODE = $Mode.ToString()
    $env:HOIST_PROBABILITY = $Probability.ToString($culture)
    $env:HOIST_SECONDS = $Seconds.ToString($culture)
    $env:HOIST_SEED = $HoistSeed.ToString()
    $env:OHT_COUNT = $OhtCount.ToString()
    $env:SIMULATION_HOURS = $Hours.ToString($culture)
    $env:SPAWN_INTERVAL_SECONDS = '0.45'
    $env:PENDING_JOB_LIMIT = '2000'
    $env:STOP_AFTER_SECONDS = '600'
    $env:STOP_WHEN_ALL_LOTS_DONE = '1'
    $env:APPLY_FIXES = '1'
    $env:ENABLE_RING_CONTROL = '1'
    $env:RING_CAPACITY_MARGIN = '2'
    [Environment]::SetEnvironmentVariable('CASE_DIRECTORY',$null,'Process')
    New-Item -ItemType Directory -Force -Path 'runs' | Out-Null
    $stamp = Get-Date -Format 'yyyyMMdd_HHmmss_fff'
    $env:RESULT_JSON_PATH = Join-Path $PSScriptRoot "runs/mode${Mode}_oht${OhtCount}_${stamp}.json"
    & $Python -X utf8 deadlock_detector.py
    if ($LASTEXITCODE -ne 0) { throw "Simulation failed: exit $LASTEXITCODE" }
} finally {
    Pop-Location
    foreach ($name in $names) { [Environment]::SetEnvironmentVariable($name,$saved[$name],'Process') }
}
