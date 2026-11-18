#pragma once
// Hand-written runtime for the generated packet headers (NOT generated). Explicit little-endian, no struct
// copies, no heap, no exceptions. The fixed-point rules here are mirrored by python/exvectr_packets/_runtime.py
// and specified in docs/schema-format.md.
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <limits>

namespace VCTR {
namespace Packets {
namespace codec {

inline void put_u8(uint8_t* p, uint8_t v) { p[0] = v; }
inline void put_i8(uint8_t* p, int8_t v) { p[0] = static_cast<uint8_t>(v); }
inline void put_u16(uint8_t* p, uint16_t v) { p[0] = uint8_t(v); p[1] = uint8_t(v >> 8); }
inline void put_i16(uint8_t* p, int16_t v) { put_u16(p, static_cast<uint16_t>(v)); }
inline void put_u32(uint8_t* p, uint32_t v) { for (int i = 0; i < 4; ++i) p[i] = uint8_t(v >> (8 * i)); }
inline void put_i32(uint8_t* p, int32_t v) { put_u32(p, static_cast<uint32_t>(v)); }
inline void put_u64(uint8_t* p, uint64_t v) { for (int i = 0; i < 8; ++i) p[i] = uint8_t(v >> (8 * i)); }
inline void put_i64(uint8_t* p, int64_t v) { put_u64(p, static_cast<uint64_t>(v)); }

inline uint8_t get_u8(const uint8_t* p) { return p[0]; }
inline int8_t get_i8(const uint8_t* p) { return static_cast<int8_t>(p[0]); }
inline uint16_t get_u16(const uint8_t* p) { return uint16_t(p[0] | (uint16_t(p[1]) << 8)); }
inline int16_t get_i16(const uint8_t* p) { return static_cast<int16_t>(get_u16(p)); }
inline uint32_t get_u32(const uint8_t* p) {
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
inline int32_t get_i32(const uint8_t* p) { return static_cast<int32_t>(get_u32(p)); }
inline uint64_t get_u64(const uint8_t* p) { return uint64_t(get_u32(p)) | (uint64_t(get_u32(p + 4)) << 32); }
inline int64_t get_i64(const uint8_t* p) { return static_cast<int64_t>(get_u64(p)); }

/// IEEE-754 binary32 little-endian. Any NaN is written as the canonical quiet NaN 0x7FC00000.
inline void put_f32(uint8_t* p, float v) {
    uint32_t u;
    if (v != v) {
        u = 0x7FC00000u;
    } else {
        memcpy(&u, &v, 4);
    }
    put_u32(p, u);
}
inline float get_f32(const uint8_t* p) {
    uint32_t u = get_u32(p);
    float v;
    memcpy(&v, &u, 4);
    return v;
}
/// binary64, canonical NaN 0x7FF8000000000000.
inline void put_f64(uint8_t* p, double v) {
    uint64_t u;
    if (v != v) {
        u = 0x7FF8000000000000ull;
    } else {
        memcpy(&u, &v, 8);
    }
    put_u64(p, u);
}
inline double get_f64(const uint8_t* p) {
    uint64_t u = get_u64(p);
    double v;
    memcpy(&v, &u, 8);
    return v;
}

/// Fixed-point encode: raw = round_half_away_from_zero(clamp(v / scale, lo, hi)). NaN and +-Inf give `invalid`.
/// lo/hi already exclude the invalid sentinel (computed by the generator).
inline int64_t fixedEncode(double v, double scale, int64_t lo, int64_t hi, int64_t invalid) {
    if (!(v == v) || v > std::numeric_limits<double>::max() || v < -std::numeric_limits<double>::max()) return invalid;
    double x = v / scale;
    if (x >= static_cast<double>(hi)) return hi;
    if (x <= static_cast<double>(lo)) return lo;
    return static_cast<int64_t>(::round(x));  // half away from zero
}
/// Fixed-point decode: the invalid sentinel gives NaN, otherwise raw * scale.
inline double fixedDecode(int64_t raw, double scale, int64_t invalid) {
    if (raw == invalid) return std::numeric_limits<double>::quiet_NaN();
    return static_cast<double>(raw) * scale;
}

} // namespace codec
} // namespace Packets
} // namespace VCTR
