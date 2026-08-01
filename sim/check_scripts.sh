#!/usr/bin/env bash
# Syntax-check every script this project ships, without running any of them.
# Cheap guard: a shell script with a syntax error fails at the worst moment,
# usually mid-flight in another terminal.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fail=0

for f in "$HERE"/*.sh "$HERE"/ros2/*.sh; do
    [ -e "$f" ] || continue
    if bash -n "$f" 2>/dev/null; then
        echo "  ok      $(basename "$f")"
    else
        echo "  SYNTAX  $(basename "$f")"; bash -n "$f"; fail=1
    fi
done

for f in "$HERE"/ros2/*.py "$HERE"/*.py; do
    [ -e "$f" ] || continue
    if python3 -m py_compile "$f" 2>/dev/null; then
        echo "  ok      $(basename "$f")"
    else
        echo "  SYNTAX  $(basename "$f")"; python3 -m py_compile "$f"; fail=1
    fi
done

exit $fail
