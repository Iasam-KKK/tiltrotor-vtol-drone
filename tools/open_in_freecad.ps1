# Open the tri-tiltrotor STEP assembly in FreeCAD, coloured by role.
#
#   powershell -File tools/open_in_freecad.ps1
#
# Regenerates the STEP export first so what you see is what params.py currently
# says, not whatever was exported last time.
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$freecad = if ($env:FREECAD) { $env:FREECAD } else { "D:\FreeCAD\bin\freecad.exe" }

if (-not (Test-Path $freecad)) {
    Write-Error "FreeCAD not found at $freecad. Set `$env:FREECAD."
}

$venv = Join-Path $root "..\..\.venv-cad\Scripts\python.exe"
if (Test-Path $venv) {
    Write-Host "regenerating STEP assembly + annotations from params.py..."
    Push-Location (Join-Path $root "cad")
    & $venv "gen_assembly_step.py" | Select-Object -Last 2
    & $venv "gen_annotations.py"   | Select-Object -Last 2
    Pop-Location
}

$macro = Join-Path $PSScriptRoot "load_assembly.FCMacro"
Write-Host "opening FreeCAD..."
Start-Process -FilePath $freecad -ArgumentList $macro
