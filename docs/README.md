# ExVectrPackets documentation

The library has two parts.

1. **Legacy hand-written packet structs and CRSF/radiolink classes** under `include/ExVectrPackets/` (used by ExVectrLink FPV and the flight software). Untouched by the schema work.
2. **Schema-generated packets** for the ExVectrNetwork stack (`schema/`, `tools/`, `gen/`, `python/`). Design: workspace `docs/network-design.md` section 3.5 and 4.7.

| Doc | Content |
|---|---|
| [schema-format.md](schema-format.md) | TOML schema, field types, fixed-point rules, generated C++ and Python API, generator usage |
| [adding-a-packet.md](adding-a-packet.md) | Step by step: add or change a packet |
| [versioning.md](versioning.md) | Append-only rule, breaking changes, schema hash |
| [telemetry-sets.md](telemetry-sets.md) | The air (radio) and USB telemetry sets, rate classes, budget |
| [packet-reference.md](packet-reference.md) | Generated: every packet, offsets, units (do not edit) |

Tests: `pio test -e native` (C++), `python3 -m unittest discover -s test/python -t .` (Python), `tools/check_legacy_headers.sh` (legacy headers still compile).
