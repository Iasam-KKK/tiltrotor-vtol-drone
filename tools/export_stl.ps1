# Export the tri-tiltrotor to STL via FreeCAD, headless.
#
#   powershell -File tools\export_stl.ps1
#
# Regenerates the STEP assembly from params.py first, so the STLs are what the
# parameters currently say and not whatever was exported last time.
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot

# freecadcmd, not freecad: the GUI build opens a window and never returns.
$freecadcmd = if ($env:FREECADCMD) { $env:FREECADCMD } else { "D:\FreeCAD\bin\freecadcmd.exe" }
if (-not (Test-Path $freecadcmd)) {
    Write-Error "freecadcmd not found at $freecadcmd. Set `$env:FREECADCMD."
}

$venv = Join-Path $root "..\..\.venv-cad\Scripts\python.exe"
if (Test-Path $venv) {
    Write-Host "regenerating STEP assembly from params.py..."
    Push-Location (Join-Path $root "cad")
    & $venv "gen_assembly_step.py" | Select-Object -Last 3
    Pop-Location
}

Write-Host "meshing in FreeCAD..."
Push-Location $root
# The mesher spams a tab-delimited percentage bar to stdout that buries the
# report. Drop those lines; keep everything else.
& $freecadcmd (Join-Path $PSScriptRoot "export_stl.py") 2>&1 |
    Where-Object { $_ -notmatch '^\s*(saving|\t|\(\d+ %\))' }
Pop-Location
