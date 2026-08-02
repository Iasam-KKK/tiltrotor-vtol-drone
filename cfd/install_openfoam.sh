#!/usr/bin/env bash
# Install ESI OpenFOAM (v2312+) into the existing Ubuntu 24.04 WSL box.
#
# ESI (openfoam.com) rather than the Foundation build (openfoam.org): the
# Foundation renamed constant/turbulenceProperties to momentumTransport and
# reworked the solver names, so a case written for one will not run on the
# other. cad/gen_cfd_case.py emits ESI dictionaries. Installing both and
# sourcing the wrong one is a confusing afternoon.
#
# Run once:  bash cfd/install_openfoam.sh
set -euo pipefail

echo "=============================================================="
echo "OpenFOAM (ESI) install"
echo "=============================================================="

if command -v simpleFoam >/dev/null 2>&1; then
    echo "simpleFoam already on PATH: $(command -v simpleFoam)"
    exit 0
fi

. /etc/os-release
if [ "${VERSION_CODENAME:-}" != "noble" ]; then
    echo "WARNING: expected Ubuntu 24.04 (noble), found ${VERSION_CODENAME:-unknown}"
fi

echo
echo "[1/3] adding the openfoam.com repository"
curl -fsSL https://dl.openfoam.com/add-debian-repo.sh | sudo bash

echo
echo "[2/3] installing (this pulls ~2 GB)"
sudo apt-get update
# Take whatever the newest packaged default is rather than pinning a version
# that will age out of the repo.
sudo apt-get install -y openfoam2412-default \
  || sudo apt-get install -y openfoam2406-default \
  || sudo apt-get install -y openfoam2312-default

echo
echo "[3/3] putting the environment on your shell"
BASHRC="$HOME/.bashrc"
if ! grep -q "openfoam.*/etc/bashrc" "$BASHRC" 2>/dev/null; then
    OFRC=$(ls -1 /usr/lib/openfoam/openfoam*/etc/bashrc 2>/dev/null | sort -V | tail -1)
    [ -n "$OFRC" ] || { echo "could not find the OpenFOAM etc/bashrc" >&2; exit 1; }
    printf '\n# OpenFOAM (ESI)\nsource %s\n' "$OFRC" >> "$BASHRC"
    echo "  added: source $OFRC"
else
    echo "  already present in ~/.bashrc"
fi

echo
echo "=============================================================="
echo "Open a NEW shell, then check:"
echo "    simpleFoam -help | head -3"
echo
echo "Also raise WSL's memory before meshing. WSL currently sees only"
echo "$(free -g | awk '/^Mem:/{print $2}') GB of your 32 GB, and snappyHexMesh wants roughly"
echo "1 GB per million cells. In Windows, create %USERPROFILE%\\.wslconfig:"
echo "    [wsl2]"
echo "    memory=24GB"
echo "    processors=18"
echo "then run  wsl --shutdown  and reopen."
echo "=============================================================="
