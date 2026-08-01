#!/usr/bin/env bash
# Which screen-capture tool is available for probe_gui_pixels.sh?
for t in import convert xwd ffmpeg scrot gnome-screenshot; do
    p=$(command -v "$t" 2>/dev/null)
    printf '  %-18s %s\n' "$t" "${p:-MISSING}"
done
echo
echo "sudo without password?"
if sudo -n true 2>/dev/null; then echo "  yes"; else echo "  NO - apt install needs your password"; fi
