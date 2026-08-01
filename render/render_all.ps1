# Render every pose and view of the tri-tiltrotor, headless, on the GPU.
#
# Blender is not on PATH; point BLENDER at the executable if it moves.
# Renders land in media/renders/ and take a few seconds each on OPTIX.
#
#   powershell -File render/render_all.ps1

$ErrorActionPreference = "Stop"
$BLENDER = if ($env:BLENDER) { $env:BLENDER } else { "D:\Blender\blender.exe" }
$SCRIPT  = Join-Path $PSScriptRoot "hero_render.py"

if (-not (Test-Path $BLENDER)) {
    Write-Error "Blender not found at $BLENDER. Set `$env:BLENDER."
}

# Regenerate the manifest first so the renders cannot lag behind params.py.
$venv = Join-Path $PSScriptRoot "..\..\..\.venv-cad\Scripts\python.exe"
if (Test-Path $venv) {
    Write-Host "regenerating assembly manifest..."
    & $venv (Join-Path $PSScriptRoot "..\cad\gen_manifest.py")
}

foreach ($pose in @("hover", "transition", "cruise")) {
    Write-Host ""
    Write-Host "=== $pose ==="
    & $BLENDER -b --factory-startup --python $SCRIPT -- `
        --pose $pose --samples 128 --views "hero,front,top,detail" |
        Select-String -Pattern "rendering|Saved|cycles device|imported|ERROR"
}

Write-Host ""
Write-Host "renders in media/renders/"
Get-ChildItem (Join-Path $PSScriptRoot "..\media\renders") -Filter *.png |
    Select-Object Name, @{n="MB";e={[math]::Round($_.Length/1MB,2)}}
