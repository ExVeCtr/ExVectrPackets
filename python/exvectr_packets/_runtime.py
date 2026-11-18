"""Hand-written runtime for the generated modules (NOT generated). Standard library only.

Mirrors gen/ExVectrPackets/gen/codec.hpp. Fixed-point rules (docs/schema-format.md):
  encode: NaN/+-Inf -> invalid sentinel; else raw = round_half_away(clamp(v / scale, lo, hi))
  decode: invalid sentinel -> NaN; else raw * scale
"""
import math
import struct


def put(buf, fmt, off, v):
    try:
        struct.pack_into(fmt, buf, off, v)
    except struct.error as e:
        raise ValueError("value %r does not fit %s: %s" % (v, fmt, e)) from None


def get(data, fmt, off):
    return struct.unpack_from(fmt, data, off)[0]


def put_f32(buf, off, v):
    v = float(v)
    if v != v:
        struct.pack_into("<I", buf, off, 0x7FC00000)
        return
    try:
        struct.pack_into("<f", buf, off, v)
    except OverflowError:
        struct.pack_into("<f", buf, off, math.copysign(math.inf, v))


def put_f64(buf, off, v):
    v = float(v)
    if v != v:
        struct.pack_into("<Q", buf, off, 0x7FF8000000000000)
        return
    struct.pack_into("<d", buf, off, v)


def round_half_away(x):
    a = abs(x)
    f = math.floor(a)
    r = f + 1 if a - f >= 0.5 else f
    return -r if x < 0 else r


def fixed_raw(v, scale, lo, hi, invalid):
    v = float(v)
    if not math.isfinite(v):
        return invalid
    x = v / scale
    if x >= hi:
        return hi
    if x <= lo:
        return lo
    return int(round_half_away(x))


def put_fixed(buf, fmt, off, v, scale, lo, hi, invalid):
    put(buf, fmt, off, fixed_raw(v, scale, lo, hi, invalid))


def get_fixed(data, fmt, off, scale, invalid):
    raw = struct.unpack_from(fmt, data, off)[0]
    if raw == invalid:
        return math.nan
    return raw * scale


def put_chars(buf, off, n, s):
    raw = s.encode("ascii")  # ValueError (UnicodeEncodeError) if not ASCII
    if len(raw) > n:
        raise ValueError("text longer than %d characters" % n)
    raw = raw.split(b"\0", 1)[0]
    buf[off:off + n] = raw + b"\0" * (n - len(raw))


def get_chars(data, off, n):
    raw = bytes(0x3F if c >= 0x80 else c for c in data[off:off + n])
    return raw.split(b"\0", 1)[0].decode("ascii")


def to_enum(cls, v):
    """Enum member for a known value, else the plain int (decoders must not fail on newer senders)."""
    try:
        return cls(v)
    except ValueError:
        return v
