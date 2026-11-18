#!/bin/sh
# Regenerates every committed generated file. Run from anywhere.
set -e
cd "$(dirname "$0")/.."
python3 tools/gen_packets.py --schema schema/common.toml --schema schema/starship.toml \
  --cpp-out gen --py-out python/exvectr_packets --doc-out docs/packet-reference.md "$@"
python3 tools/gen_packets.py --schema test/schema/test_packets.toml \
  --cpp-namespace VCTR::PacketsTest --py-runtime exvectr_packets._runtime \
  --cpp-out test/generated/cpp --py-out test/generated/py/exvectr_test_packets "$@"
