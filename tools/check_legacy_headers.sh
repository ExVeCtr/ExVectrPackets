#!/bin/sh
# (<cmath>/<algorithm> are pre-included because some legacy headers forgot them; that is pre-existing.)
# Syntax-checks every legacy hand-written header under include/ (used by FPV and the flight software), to show the
# additive generator work did not break them. Needs ExVectrCore and ExVectrMath include dirs; point at them with
#   CORE_INC=... MATH_INC=... tools/check_legacy_headers.sh
# Defaults: ExVectrLink/lib (Core) and flight-software/lib (Math) in the workspace. Skips (exit 0, message) if they are not found.
cd "$(dirname "$0")/.."
WS="$PWD"
while [ "$WS" != "/" ] && [ ! -d "$WS/flight-software" ]; do WS="$(dirname "$WS")"; done
CORE_INC=${CORE_INC:-$WS/ExVectrLink/lib/ExVectrCore/include}
MATH_INC=${MATH_INC:-$WS/flight-software/lib/ExVectrMath/include}
[ -d "$CORE_INC" ] && [ -d "$MATH_INC" ] || { echo "skipped: set CORE_INC and MATH_INC"; exit 0; }
fail=0
for h in $(find include -name '*.hpp' | sort); do
  if printf "#include <cmath>\n#include <algorithm>\n#include \"${h#include/}\"\n" | g++ -std=gnu++20 -fsyntax-only -x c++ -Iinclude -I"$CORE_INC" -I"$MATH_INC" - 2>/tmp/legacy_err.txt; then :; else
    echo "FAIL $h"; head -3 /tmp/legacy_err.txt; fail=1
  fi
done
[ $fail = 0 ] && echo "legacy headers: all compile"
exit $fail
