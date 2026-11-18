# Schema format and generator

Files: `schema/common.toml` (link, control and system packets, ports 0x01-0x1F, heartbeat payload) and `schema/starship.toml` (vehicle telemetry, request action enum). `tools/gen_packets.py` (Python 3.11+, standard library only) turns them into:

| Output | Location | Committed |
|---|---|---|
| C++ headers, namespace `VCTR::Packets` | `gen/ExVectrPackets/gen/<schema>.hpp`, `all.hpp` (includes all, defines `kSchemaHash`) | yes |
| Python package `exvectr_packets` | `python/exvectr_packets/<schema>.py`, `__init__.py` (registry, `SCHEMA_HASH`) | yes |
| Packet reference | `docs/packet-reference.md` | yes |

Hand-written runtime (not generated): `gen/ExVectrPackets/gen/codec.hpp` and `python/exvectr_packets/_runtime.py`. They implement the byte and fixed-point rules below and must stay in step.

Regenerate everything with `tools/regen.sh`. Check without writing: add `--check` (exit 1 on any difference). Both test suites run the check, so forgetting to regenerate fails CI. The test-only schema `test/schema/test_packets.toml` is generated into `test/generated/` (namespace `VCTR::PacketsTest`); it exercises every field kind.

C++ consumers add `gen/` to the include path (`-I<lib>/gen`, PlatformIO: `build_flags = -I lib/ExVectrPackets/gen`) and `#include "ExVectrPackets/gen/all.hpp"`. Python consumers put `python/` on `sys.path`.

## Schema file layout

```toml
[schema]
name = "starship"
doc = "..."

[enum.RequestAction]            # PascalCase name, UPPER_SNAKE members
type = "u8"                      # underlying wire type (default u8)
values = { NONE = 0, SAVE_CONFIG = 8 }

[packet.RC_STATE]               # UPPER_SNAKE name -> C++/Python class RcState
port = 0x10                      # 0x01..0xFE, unique over all files. Omit and set heartbeat = true for the heartbeat payload.
qos = "control"                  # control | link | event | telemetry (the 2-bit QoS of the network frame)
latest = true                    # latest-wins (state) or FIFO (events)
rate_air = 25                    # optional, Hz on the radio (ports >= 0x80 must not have one)
rate_usb = 50                    # optional, Hz on USB
consts = { CH_MIN = -1000 }      # optional named integer constants
doc = "..."
fields = [ { name = "flags", type = "u8", bits = { SOURCE_OK = 0 } }, ... ]
```

Ports follow the design port map: 0x01-0x0F link/system, 0x10-0x1F control, 0x20-0x7F radio telemetry (air-eligible), 0x80-0xEF USB-only telemetry (dropped by the air and UART egress filters), 0xF0-0xFE experimental. `air eligible` is derived: `port < 0x80`. The generator rejects: duplicate ports or field names, a payload above 234 bytes, a `bytes_tail` that is not last, a non-optional field after an optional one, a bad `fixed` definition, `rate_air` on a port >= 0x80.

## Field types

Common keys: `name` (snake_case), `type`, `count` (fixed-length array), `unit` and `doc` (documentation only), `optional`, `default`, `bits`.

| type | Wire | C++ | Python |
|---|---|---|---|
| `u8 i8 u16 i16 u32 i32 u64 i64` | little-endian integer | matching `intN_t` | `int` |
| `f32`, `f64` | IEEE-754 LE. Any NaN is written as the canonical quiet NaN (`7FC00000`, `7FF8000000000000`). | `float`, `double` | `float` |
| `bool` | u8, 0/1 (any non-zero decodes true) | `bool` | `bool` |
| `enum` (`enum = "Name"`) | the enum's integer type | `enum class Name` (members `kPascalCase`) | `enum.IntEnum` (members `UPPER_SNAKE`); an unknown value decodes as a plain `int`, never an error |
| `char` (`count = n`) | n bytes ASCII, NUL-padded, not necessarily terminated | `char name[n]` | `str` |
| `bytes_tail` (`max = n`, optional `text = true`) | rest of the payload, last field only | `uint8_t name[n]` plus `uint16_t name_len`, and `set_name(const char*)` for text | `bytes`, or `str` (UTF-8) with `text = true` |
| `fixed` | integer scaled to SI, see below | `float` (`double` with `as = "f64"`) | `float` |

`bits = { NAME = n }` on an integer field creates the constants `FIELD_NAME` (Python class attribute, value `1 << n`) and `kFieldName` (C++). The field itself stays a plain integer mask.

Text: `char` encode refuses non-ASCII in Python (`ValueError`); C++ writes `?` for bytes >= 0x80, and both decoders map bytes >= 0x80 to `?`, so decoded text is always ASCII. `bytes_tail` text is UTF-8 and decoded with replacement characters in Python.

## Decoded values are SI

Physical quantities decode to rad, rad/s, m, m/s, s, N, V, K, Pa, T. Never degrees or scaled integers. Wire scaling exists only to save air bytes and is defined by the schema. Exceptions, deliberately: `t_us` / `t_ns` timestamps are integer counters (the receiver unwraps `t_us`), counters, bit masks, ids, and dimensionless percentages and dB values stay integers.

## Fixed-point (`type = "fixed"`)

`wire` is an integer type of at most 32 bits. Give exactly one of `scale` (SI units per LSB, positive) or `limit` (signed wire only, full scale: `scale = limit / maxraw`, so `limit = 1.0` on `i16` maps +-1.0 to +-32767). Optional `invalid` is the sentinel raw value, default the most negative value (signed) or the maximum (unsigned). A sentinel in the middle of the range is rejected.

The valid raw range is the wire range minus the sentinel when it sits at an end: `i16`: -32767..32767, `u16`: 0..65534, `u16` with `invalid = 0`: 1..65535.

Encode (identical in C++ `codec::fixedEncode` and Python `_runtime.fixed_raw`):
1. NaN, +Inf and -Inf give the sentinel.
2. `x = v / scale` (IEEE double division).
3. `x` at or beyond the valid range saturates to its end.
4. Otherwise `raw = round(x)`, half away from zero (`2.5 -> 3`, `-2.5 -> -3`). Not banker's rounding, not truncation.

Decode: the sentinel gives NaN, otherwise `raw * scale` (double, narrowed to float in C++). Decoders must treat NaN as "not available". A value that rounds to the sentinel in the middle of the range (for example `0.0` for a field with `invalid = 0`) saturates to the nearest valid raw value (1), never to the sentinel.

Differences from design 3.5.1: Inf maps to the sentinel exactly as NaN (as written there), the formula is `v / scale` instead of `v * 32767 / limit` (same result, one code path for `limit` and `scale`), and `invalid` may be set per field.

## Optional fields (append-only evolution)

A field marked `optional = true` (all following fields must be optional too, and there can be no `bytes_tail`) was added after the first release. `kMinSize` / `MIN_SIZE` is the end of the last non-optional field. Decode rules, in both languages:
- shorter than `MIN_SIZE`: decode fails (C++ returns false and leaves the object untouched, Python raises `ValueError`);
- longer than `MAX_SIZE`: accepted, extra bytes ignored (a newer sender);
- between: optional fields that do not fit get `default` (integers, bool, enum, default 0) or NaN (`f32`, `f64`, `fixed`).
Encode always writes every field.

## Generated C++

```cpp
struct RcState {
  static constexpr uint8_t kPort = 0x10;  kQos (0 control, 1 link, 2 event, 3 telemetry);  kLatest;
  kAirEligible;  kAirRateHz;  kUsbRateHz;  kMaxSize;  kMinSize;  kIsHeartbeat;
  // constants (kFlagsSourceOk, kChInvalid, ...), members named as in the schema
  size_t encode(uint8_t* out, size_t cap) const;          // bytes written, 0 on failure
  bool   decode(const uint8_t* in, size_t len);           // false if malformed, object unchanged
  template <class V> void visit(V&);                      // scalar/array/chars/tail visitor (tests, tooling)
};
```

This is exactly the shape `VCTR::network::PacketPort<T>` (ExVectrNetwork) requires (`kMaxSize <= 234`, `encode`, `decode`); `kPort`, `kQos` and `kLatest` are the values to pass to the constructor. Properties: header-only, no heap, no exceptions, no RTTI, no struct memcpy: every field is written byte by byte (little-endian) at a constant offset, with bounds checked once up front. Encode never writes past `cap`.

## Generated Python

Each packet is a `@dataclasses.dataclass` with the schema field names, class attributes `PORT`, `QOS`, `LATEST`, `AIR_ELIGIBLE`, `AIR_RATE_HZ`, `USB_RATE_HZ`, `MAX_SIZE`, `MIN_SIZE`, constants, `encode() -> bytes` and `classmethod decode(data)`. `exvectr_packets.PACKETS_BY_PORT`, `PACKETS_BY_NAME` and `SCHEMA_HASH` come from the registry. Encode raises `ValueError` for out-of-range integers, wrong array lengths, over-long text. No dependencies beyond the standard library.

```python
import exvectr_packets as p
raw = p.RcState(flags=p.RcState.FLAGS_SOURCE_OK | p.RcState.FLAGS_MOTOR_ENABLE, ch=[0] * 16).encode()
pkt = p.PACKETS_BY_PORT[0x10].decode(raw)
```

## Shared test vectors

`test/vectors/packets.json` is written by `tools/make_vectors.py`, which builds the bytes with plain `struct` and explicit raw integers, independent of the generator, so the vectors check the generated code against the spec. Sections: `cases` (values -> hex, and hex -> decoded), `decode_only` (older/newer senders, text replacement) and `decode_fail`. JSON `null` means NaN, the strings `"inf"`/`"-inf"` mean infinity, bytes tails are `{"hex": ...}`. Both the C++ and the Python suite replay every case.
