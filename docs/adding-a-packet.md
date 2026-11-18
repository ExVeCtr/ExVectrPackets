# Adding or changing a packet

1. **Decide the port and the set.** Radio telemetry (ports 0x20-0x7F) costs airtime: check the budget in [telemetry-sets.md](telemetry-sets.md). USB-only debug data goes to 0x80-0xEF. Control and link packets live in `common.toml`, vehicle data in `starship.toml`.
2. **Edit the schema.**
   - New packet: add a `[packet.NAME]` table with the next free port, `qos`, `latest`, rates, and the fields in wire order.
   - New field in an existing packet: append it at the end with `optional = true` (see [versioning.md](versioning.md)).
   - Anything else (reorder, retype, rescale, shrink, remove) is a breaking change: new packet, new port, retire the old one.
   - Use SI units. Prefer `f32` on USB. Use `fixed` only to save air bytes and always set `unit` and a sensible `invalid`.
3. **Regenerate:** `tools/regen.sh`. Commit the schema and every regenerated file together (`gen/`, `python/`, `docs/packet-reference.md`, `test/generated/`).
4. **Add vectors:** add a case to `tools/make_vectors.py` with explicit raw bytes (not produced by the generator), run `python3 tools/make_vectors.py`, commit `test/vectors/packets.json`. Cover the NaN/saturation edge of every new fixed field and a short-payload decode for every new optional field.
5. **Register the C++ test type:** add one `OPS("NAME", Type)` line to `kOps` in `test/test_gen/test_main.cpp`.
6. **Run both suites:** `~/.platformio/penv/bin/pio test -e native`, `python3 -m unittest discover -s test/python -t .`. They fail if any generated file differs from a fresh generation.
7. **Use it:** in C++ `PacketPort<Type> port(node, mux, Type::kPort, dst, static_cast<wire::Qos>(Type::kQos), Type::kLatest);`.
8. If the change affects both ends of a link (vehicle and ground station), each project must pin an ExVectrPackets version that carries it. Heartbeats expose a mismatch through the schema hash (the receiver sets `INCOMPATIBLE`).
