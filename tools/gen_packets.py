#!/usr/bin/env python3
"""ExVectrPackets generator: TOML schema -> C++ headers + Python package. Standard library only (Python 3.11+).

Usage:
  gen_packets.py --schema a.toml --schema b.toml --cpp-out DIR --py-out DIR [--cpp-namespace NS]
                 [--py-runtime MODULE] [--check]
--check generates in memory and compares with the files on disk (exit 1 on any difference).
Format: docs/schema-format.md. Output is deterministic (no timestamps, sorted where order is irrelevant).
"""
import argparse
import json
import keyword
import math
import re
import sys
import tomllib
from pathlib import Path

MAX_PAYLOAD = 234  # largest transport payload (design 4.4)
INT_TYPES = {"u8": (1, False), "i8": (1, True), "u16": (2, False), "i16": (2, True),
             "u32": (4, False), "i32": (4, True), "u64": (8, False), "i64": (8, True)}
QOS = {"control": 0, "link": 1, "event": 2, "telemetry": 3}
STRUCT_CODE = {"u8": "B", "i8": "b", "u16": "H", "i16": "h", "u32": "I", "i32": "i", "u64": "Q", "i64": "q"}
CPP_INT = {"u8": "uint8_t", "i8": "int8_t", "u16": "uint16_t", "i16": "int16_t",
           "u32": "uint32_t", "i32": "int32_t", "u64": "uint64_t", "i64": "int64_t"}


class SchemaError(Exception):
    pass


def pascal(name):
    return "".join(p.capitalize() for p in name.lower().split("_"))


def fnv1a32(data: bytes) -> int:
    h = 0x811C9DC5
    for b in data:
        h = ((h ^ b) * 0x01000193) & 0xFFFFFFFF
    return h


def strip_docs(o):
    if isinstance(o, dict):
        return {k: strip_docs(v) for k, v in o.items() if k not in ("doc", "unit")}
    if isinstance(o, list):
        return [strip_docs(v) for v in o]
    return o


def schema_hash(files):
    """FNV-1a-32 over a canonical JSON serialisation of all files (doc/unit text and comments excluded, files
    sorted by stem, table keys sorted, field order kept)."""
    canon = {stem: strip_docs(data) for stem, data in sorted(files.items())}
    return fnv1a32(json.dumps(canon, sort_keys=True, separators=(",", ":")).encode())


# ------------------------------------------------------------------ model

class Field:
    pass


def fixed_range(wire, invalid):
    size, signed = INT_TYPES[wire]
    lo = -(1 << (size * 8 - 1)) if signed else 0
    hi = (1 << (size * 8 - 1)) - 1 if signed else (1 << (size * 8)) - 1
    if invalid == lo:
        lo += 1
    elif invalid == hi:
        hi -= 1
    elif not (lo < invalid < hi):
        raise SchemaError("invalid sentinel out of range")
    return lo, hi


def parse_file(stem, data):
    enums = {}
    for ename, e in data.get("enum", {}).items():
        t = e.get("type", "u8")
        if t not in INT_TYPES:
            raise SchemaError(f"{stem}: enum {ename}: bad type")
        vals = e["values"]
        if len(set(vals.values())) != len(vals):
            raise SchemaError(f"{stem}: enum {ename}: duplicate values")
        size, signed = INT_TYPES[t]
        for k, v in vals.items():
            if not re.fullmatch(r"[A-Z][A-Z0-9_]*", k):
                raise SchemaError(f"{stem}: enum {ename}: member {k} must be UPPER_SNAKE")
            lim = 1 << (size * 8 - (1 if signed else 0))
            if not ((-lim if signed else 0) <= v < lim):
                raise SchemaError(f"{stem}: enum {ename}.{k} out of range")
        enums[ename] = {"type": t, "values": dict(vals)}
    packets = []
    for pname, p in data.get("packet", {}).items():
        packets.append(parse_packet(stem, pname, p, enums))
    return {"stem": stem, "enums": enums, "packets": packets, "doc": data.get("schema", {}).get("doc", "")}


def parse_packet(stem, pname, p, enums):
    if not re.fullmatch(r"[A-Z][A-Z0-9_]*", pname):
        raise SchemaError(f"packet {pname}: name must be UPPER_SNAKE")
    pk = {"name": pname, "cls": pascal(pname), "doc": p.get("doc", ""), "heartbeat": bool(p.get("heartbeat", False)),
          "port": p.get("port"), "qos": p["qos"], "latest": bool(p["latest"]),
          "rate_air": int(p.get("rate_air", 0)), "rate_usb": int(p.get("rate_usb", 0)),
          "consts": dict(p.get("consts", {})), "fields": []}
    if pk["qos"] not in QOS:
        raise SchemaError(f"{pname}: bad qos")
    if pk["heartbeat"]:
        if pk["port"] is not None:
            raise SchemaError(f"{pname}: heartbeat packets have no port")
    else:
        port = pk["port"]
        if not isinstance(port, int) or not (0x01 <= port <= 0xFE):
            raise SchemaError(f"{pname}: port must be 0x01..0xFE")
        if port >= 0x80 and pk["rate_air"]:
            raise SchemaError(f"{pname}: ports >= 0x80 are not air-eligible, rate_air must be 0")
    pk["air"] = (not pk["heartbeat"]) and pk["port"] < 0x80
    off = 0
    seen = set()
    min_end = None
    opt_seen = False
    names = set()
    for fd in p["fields"]:
        f = Field()
        f.name = fd["name"]
        if not re.fullmatch(r"[a-z][a-z0-9_]*", f.name) or keyword.iskeyword(f.name) or f.name in names:
            raise SchemaError(f"{pname}: bad or duplicate field name {f.name!r}")
        names.add(f.name)
        f.type = fd["type"]
        f.count = fd.get("count")
        f.is_array = f.count is not None
        n = f.count if f.is_array else 1
        f.doc = fd.get("doc", "")
        f.unit = fd.get("unit", "")
        f.bits = fd.get("bits", {})
        f.optional = bool(fd.get("optional", False))
        f.default = fd.get("default", 0)
        f.valid = fd.get("valid")            # optional: list of allowed raw integer values (decode/encode reject others)
        f.min_value = fd.get("min_value")    # optional: smallest allowed raw integer value
        f.max = None
        f.text = False
        if f.is_array and (not isinstance(n, int) or n < 1):
            raise SchemaError(f"{pname}.{f.name}: bad count")
        if f.optional:
            opt_seen = True
        elif opt_seen:
            raise SchemaError(f"{pname}.{f.name}: non-optional field after an optional one")
        t = f.type
        if t in INT_TYPES:
            f.size = INT_TYPES[t][0]
            f.cpp = CPP_INT[t]
        elif t == "f32":
            f.size, f.cpp = 4, "float"
        elif t == "f64":
            f.size, f.cpp = 8, "double"
        elif t == "bool":
            f.size, f.cpp = 1, "bool"
        elif t == "enum":
            en = fd.get("enum")
            if en not in enums:
                raise SchemaError(f"{pname}.{f.name}: unknown enum {en!r} (enums must be in the same file)")
            f.enum = en
            f.wire = enums[en]["type"]
            f.size = INT_TYPES[f.wire][0]
            f.cpp = en
            if fd.get("strict"):                 # strict enum: only the declared members are accepted
                f.valid = sorted(enums[en]["values"].values())
        elif t == "char":
            if not f.is_array:
                raise SchemaError(f"{pname}.{f.name}: char needs count")
            f.size, f.cpp = 1, "char"
        elif t == "fixed":
            f.wire = fd["wire"]
            if f.wire not in INT_TYPES or INT_TYPES[f.wire][0] > 4:
                raise SchemaError(f"{pname}.{f.name}: fixed wire must be an integer type up to 32 bits")
            size, signed = INT_TYPES[f.wire]
            f.size, f.cpp = size, ("double" if fd.get("as") == "f64" else "float")
            if ("limit" in fd) == ("scale" in fd):
                raise SchemaError(f"{pname}.{f.name}: give exactly one of limit or scale")
            if "invalid" in fd:
                f.invalid = int(fd["invalid"])
            else:
                f.invalid = -(1 << (size * 8 - 1)) if signed else (1 << (size * 8)) - 1
            f.lo, f.hi = fixed_range(f.wire, f.invalid)
            if "limit" in fd:
                if not signed:
                    raise SchemaError(f"{pname}.{f.name}: limit needs a signed wire type, use scale")
                f.scale = float(fd["limit"]) / f.hi
            else:
                f.scale = float(fd["scale"])
            if not (f.scale > 0 and math.isfinite(f.scale)):
                raise SchemaError(f"{pname}.{f.name}: scale must be positive")
        elif t == "bytes_tail":
            f.max = int(fd["max"])
            f.text = bool(fd.get("text", False))
            f.size, f.cpp = 1, "uint8_t"
            if f.is_array or f.optional:
                raise SchemaError(f"{pname}.{f.name}: bytes_tail cannot be an array or optional")
        else:
            raise SchemaError(f"{pname}.{f.name}: unknown type {t!r}")
        if (f.valid is not None or f.min_value is not None) and (
                f.is_array or f.optional or t not in INT_TYPES and t != "enum"):
            raise SchemaError(f"{pname}.{f.name}: valid/min_value need a scalar, non-optional integer or enum field")
        if f.valid is not None and (not isinstance(f.valid, list) or not f.valid
                                    or not all(isinstance(x, int) for x in f.valid)):
            raise SchemaError(f"{pname}.{f.name}: valid must be a non-empty list of integers")
        if f.bits and (t not in INT_TYPES or f.is_array):
            raise SchemaError(f"{pname}.{f.name}: bits need a scalar integer field")
        for b, v in f.bits.items():
            if not re.fullmatch(r"[A-Z][A-Z0-9_]*", b) or not (0 <= v < f.size * 8):
                raise SchemaError(f"{pname}.{f.name}: bad bit {b}")
        f.offset = off
        f.total = f.max if t == "bytes_tail" else f.size * n
        f.n = n
        off += f.total
        if t == "bytes_tail" and fd is not p["fields"][-1]:
            raise SchemaError(f"{pname}.{f.name}: bytes_tail must be the last field")
        if not f.optional:
            min_end = off if t != "bytes_tail" else f.offset
        pk["fields"].append(f)
    pk["max_size"] = off
    pk["min_size"] = min_end or 0
    if pk["max_size"] > MAX_PAYLOAD:
        raise SchemaError(f"{pname}: {pk['max_size']} bytes exceeds {MAX_PAYLOAD}")
    for k in pk["consts"]:
        if not re.fullmatch(r"[A-Z][A-Z0-9_]*", k):
            raise SchemaError(f"{pname}: const {k} must be UPPER_SNAKE")
    return pk


def load(paths):
    files, parsed, ports = {}, [], {}
    for path in paths:
        path = Path(path)
        with open(path, "rb") as fh:
            data = tomllib.load(fh)
        files[path.stem] = data
        parsed.append(parse_file(path.stem, data))
    for pf in parsed:
        for pk in pf["packets"]:
            if pk["port"] is not None:
                if pk["port"] in ports:
                    raise SchemaError(f"port 0x{pk['port']:02X} used by {pk['name']} and {ports[pk['port']]}")
                ports[pk["port"]] = pk["name"]
    return files, parsed


# ------------------------------------------------------------------ C++

def lit(x):  # double literal, exact
    return repr(float(x))


def cpp_scalar_put(f, dst, val):
    t = f.type
    if t in INT_TYPES:
        return f"codec::put_{t}({dst}, {val});"
    if t == "f32":
        return f"codec::put_f32({dst}, {val});"
    if t == "f64":
        return f"codec::put_f64({dst}, {val});"
    if t == "bool":
        return f"codec::put_u8({dst}, {val} ? 1 : 0);"
    if t == "enum":
        return f"codec::put_{f.wire}({dst}, static_cast<{CPP_INT[f.wire]}>({val}));"
    if t == "fixed":
        return (f"codec::put_{f.wire}({dst}, static_cast<{CPP_INT[f.wire]}>(codec::fixedEncode({val}, {lit(f.scale)}, "
                f"{f.lo}LL, {f.hi}LL, {f.invalid}LL)));")
    raise AssertionError(t)


def cpp_scalar_get(f, src):
    t = f.type
    if t in INT_TYPES:
        return f"codec::get_{t}({src})"
    if t in ("f32", "f64"):
        return f"codec::get_{t}({src})"
    if t == "bool":
        return f"(codec::get_u8({src}) != 0)"
    if t == "enum":
        return f"static_cast<{f.enum}>(codec::get_{f.wire}({src}))"
    if t == "fixed":
        return f"static_cast<{f.cpp}>(codec::fixedDecode(codec::get_{f.wire}({src}), {lit(f.scale)}, {f.invalid}LL))"
    raise AssertionError(t)


def cpp_valid_cond(f, expr):
    """C++ condition that is true when the field value is allowed (None if the field is unconstrained)."""
    parts = []
    v = "static_cast<int64_t>(%s)" % expr
    if f.valid is not None:
        parts.append("(" + " || ".join("%s == %dLL" % (v, x) for x in f.valid) + ")")
    if f.min_value is not None:
        parts.append("%s >= %dLL" % (v, f.min_value))
    return " && ".join(parts) if parts else None


def py_valid_cond(f, expr):
    parts = []
    if f.valid is not None:
        parts.append("int(%s) in (%s,)" % (expr, ", ".join(str(x) for x in f.valid)))
    if f.min_value is not None:
        parts.append("int(%s) >= %d" % (expr, f.min_value))
    return " and ".join(parts) if parts else None


def cpp_default(f):
    if f.type in ("f32", "f64", "fixed"):
        return "std::numeric_limits<%s>::quiet_NaN()" % f.cpp
    if f.type == "bool":
        return "false" if not f.default else "true"
    if f.type == "enum":
        return f"static_cast<{f.enum}>({int(f.default)})"
    return str(int(f.default))


def gen_cpp_file(pf, ns, codec_inc):
    L = []
    w = L.append
    w("// GENERATED by tools/gen_packets.py from schema/%s.toml. DO NOT EDIT." % pf["stem"])
    w("#pragma once")
    w("#include <stddef.h>")
    w("#include <stdint.h>")
    w("#include <limits>")
    w('#include "%s"' % codec_inc)
    w("")
    w("namespace %s {" % ns)
    w("")
    if ns != "VCTR::Packets":
        w("namespace codec = ::VCTR::Packets::codec;")
        w("")
    for en, e in pf["enums"].items():
        w("enum class %s : %s {" % (en, CPP_INT[e["type"]]))
        for k, v in e["values"].items():
            w("    k%s = %d," % (pascal(k), v))
        w("};")
        w("")
    for pk in pf["packets"]:
        w("/// %s" % pk["doc"])
        w("struct %s {" % pk["cls"])
        if pk["heartbeat"]:
            w("    static constexpr bool kIsHeartbeat = true;")
        else:
            w("    static constexpr uint8_t kPort = 0x%02X;" % pk["port"])
            w("    static constexpr bool kIsHeartbeat = false;")
        w("    static constexpr uint8_t kQos = %d; // %s (Control 0, Link 1, Event 2, Telemetry 3)" % (QOS[pk["qos"]], pk["qos"]))
        w("    static constexpr bool kLatest = %s;" % ("true" if pk["latest"] else "false"))
        w("    static constexpr bool kAirEligible = %s;" % ("true" if pk["air"] else "false"))
        w("    static constexpr uint16_t kAirRateHz = %d;" % pk["rate_air"])
        w("    static constexpr uint16_t kUsbRateHz = %d;" % pk["rate_usb"])
        w("    static constexpr size_t kMaxSize = %d;" % pk["max_size"])
        w("    static constexpr size_t kMinSize = %d;" % pk["min_size"])
        for k, v in pk["consts"].items():
            w("    static constexpr int32_t k%s = %d;" % (pascal(k), v))
        for f in pk["fields"]:
            for b, v in f.bits.items():
                w("    static constexpr %s k%s = %s(1u << %d);" % (f.cpp, pascal(f.name + "_" + b), f.cpp, v))
        w("")
        for f in pk["fields"]:
            c = ""
            if f.unit or f.doc:
                c = " // " + (("[%s] " % f.unit) if f.unit else "") + f.doc
            if f.type == "bytes_tail":
                w("    uint8_t %s[%d] = {};%s" % (f.name, f.max, c))
                w("    uint16_t %s_len = 0;" % f.name)
            elif f.type == "char":
                w("    char %s[%d] = {};%s" % (f.name, f.n, c))
            elif f.is_array:
                w("    %s %s[%d] = {};%s" % (f.cpp, f.name, f.n, c))
            else:
                init = "{}" if f.type != "bool" else "false"
                w("    %s %s = %s;%s" % (f.cpp, f.name, init, c))
        w("")
        # encode
        total = pk["max_size"]
        tail = pk["fields"][-1] if pk["fields"] and pk["fields"][-1].type == "bytes_tail" else None
        w("    /// Bytes written, 0 on failure (null buffer or too small).")
        w("    size_t encode(uint8_t* out, size_t cap) const {")
        if tail:
            w("        const size_t total = %d + %s_len;" % (tail.offset, tail.name))
            w("        if (out == nullptr || cap < total || %s_len > %d) return 0;" % (tail.name, tail.max))
        else:
            w("        constexpr size_t total = %d;" % total)
            w("        if (out == nullptr || cap < total) return 0;")
        for f in pk["fields"]:
            c = cpp_valid_cond(f, f.name) if not f.is_array else None
            if c:
                w("        if (!(%s)) return 0;  // value not allowed" % c)
        for f in pk["fields"]:
            if f.type == "bytes_tail":
                w("        for (size_t i = 0; i < %s_len; ++i) out[%d + i] = %s[i];" % (f.name, f.offset, f.name))
            elif f.type == "char":
                w("        { bool end = false; for (size_t i = 0; i < %d; ++i) { char c = end ? '\\0' : %s[i]; if (c == '\\0') end = true; "
                  "out[%d + i] = static_cast<uint8_t>(static_cast<unsigned char>(c) >= 0x80 ? '?' : c); } }" % (f.n, f.name, f.offset))
            elif f.is_array:
                w("        for (size_t i = 0; i < %d; ++i) %s" % (f.n, cpp_scalar_put(f, "out + %d + i * %d" % (f.offset, f.size), f.name + "[i]")))
            else:
                w("        " + cpp_scalar_put(f, "out + %d" % f.offset, f.name))
        w("        return total;")
        w("    }")
        w("")
        w("    /// False if the payload is shorter than kMinSize (or a tail is too long). Longer payloads are accepted (extra")
        w("    /// bytes ignored); optional fields missing from shorter payloads get their default/NaN.")
        w("    bool decode(const uint8_t* in, size_t len) {")
        w("        if (in == nullptr || len < kMinSize) return false;")
        w("        %s t;" % pk["cls"])
        for f in pk["fields"]:
            end = f.offset + f.total
            if f.type == "bytes_tail":
                w("        { const size_t n = len - %d; if (n > %d) return false; t.%s_len = static_cast<uint16_t>(n); "
                  "for (size_t i = 0; i < n; ++i) t.%s[i] = in[%d + i]; }" % (f.offset, f.max, f.name, f.name, f.offset))
                continue
            if f.optional:
                w("        if (len >= %d) {" % end)
                ind = "            "
            else:
                ind = "        "
            if f.type == "char":
                w(ind + "for (size_t i = 0; i < %d; ++i) { uint8_t c = in[%d + i]; t.%s[i] = static_cast<char>(c >= 0x80 ? '?' : c); }" % (f.n, f.offset, f.name))
            elif f.is_array:
                w(ind + "for (size_t i = 0; i < %d; ++i) t.%s[i] = %s;" % (f.n, f.name, cpp_scalar_get(f, "in + %d + i * %d" % (f.offset, f.size))))
            else:
                w(ind + "t.%s = %s;" % (f.name, cpp_scalar_get(f, "in + %d" % f.offset)))
                c = cpp_valid_cond(f, "t." + f.name)
                if c:
                    w(ind + "if (!(%s)) return false;  // value not allowed: whole frame rejected" % c)
            if f.optional:
                w("        } else {")
                if f.is_array:
                    w("            for (size_t i = 0; i < %d; ++i) t.%s[i] = %s;" % (f.n, f.name, cpp_default(f)))
                else:
                    w("            t.%s = %s;" % (f.name, cpp_default(f)))
                w("        }")
        w("        *this = t;")
        w("        return true;")
        w("    }")
        w("")
        if tail and tail.text:
            w("    /// Sets the text from a C string, truncating to the capacity.")
            w("    void set_%s(const char* s) {" % tail.name)
            w("        size_t n = 0;")
            w("        while (s != nullptr && s[n] != '\\0' && n < %d) { %s[n] = static_cast<uint8_t>(s[n]); ++n; }" % (tail.max, tail.name))
            w("        %s_len = static_cast<uint16_t>(n);" % tail.name)
            w("    }")
            w("")
        w("    /// Calls v.scalar/array/chars/tail(name, member) for every field (used by tests and tooling).")
        w("    template <class V>")
        w("    void visit(V& v) {")
        for f in pk["fields"]:
            if f.type == "bytes_tail":
                w('        v.tail("%s", %s, %s_len);' % (f.name, f.name, f.name))
            elif f.type == "char":
                w('        v.chars("%s", %s);' % (f.name, f.name))
            elif f.is_array:
                w('        v.array("%s", %s);' % (f.name, f.name))
            else:
                w('        v.scalar("%s", %s);' % (f.name, f.name))
        w("        (void)v;")
        w("    }")
        w("};")
        w("")
    w("} // namespace %s" % ns)
    w("")
    return "\n".join(L)


def gen_cpp_all(parsed, ns, h):
    L = ["// GENERATED by tools/gen_packets.py. DO NOT EDIT.", "#pragma once", "#include <stdint.h>"]
    for pf in parsed:
        L.append('#include "ExVectrPackets/gen/%s.hpp"' % pf["stem"])
    L += ["", "namespace %s {" % ns, "", "/// FNV-1a-32 over the canonical serialisation of all schema files (heartbeat schema hash).",
          "constexpr uint32_t kSchemaHash = 0x%08Xu;" % h, "", "} // namespace %s" % ns, ""]
    return "\n".join(L)


# ------------------------------------------------------------------ Python

def py_default(f):
    if f.type in ("f32", "f64", "fixed"):
        return "float('nan')"
    if f.type == "bool":
        return "bool(%d)" % int(bool(f.default))
    if f.type == "enum":
        return str(int(f.default))
    return str(int(f.default))


def py_put(f, off, val):
    t = f.type
    fmt = "<" + STRUCT_CODE.get(t, STRUCT_CODE.get(getattr(f, "wire", ""), ""))
    if t in INT_TYPES:
        return f"_rt.put(b, '{fmt}', {off}, {val})"
    if t == "f32":
        return f"_rt.put_f32(b, {off}, {val})"
    if t == "f64":
        return f"_rt.put_f64(b, {off}, {val})"
    if t == "bool":
        return f"_rt.put(b, '<B', {off}, 1 if {val} else 0)"
    if t == "enum":
        return f"_rt.put(b, '{fmt}', {off}, int({val}))"
    if t == "fixed":
        return f"_rt.put_fixed(b, '{fmt}', {off}, {val}, {lit(f.scale)}, {f.lo}, {f.hi}, {f.invalid})"
    raise AssertionError(t)


def py_get(f, off):
    t = f.type
    fmt = "<" + STRUCT_CODE.get(t, STRUCT_CODE.get(getattr(f, "wire", ""), ""))
    if t in INT_TYPES:
        return f"_rt.get(data, '{fmt}', {off})"
    if t in ("f32", "f64"):
        return f"_rt.get(data, '<{'f' if t == 'f32' else 'd'}', {off})"
    if t == "bool":
        return f"(_rt.get(data, '<B', {off}) != 0)"
    if t == "enum":
        return f"_rt.to_enum({f.enum}, _rt.get(data, '{fmt}', {off}))"
    if t == "fixed":
        return f"_rt.get_fixed(data, '{fmt}', {off}, {lit(f.scale)}, {f.invalid})"
    raise AssertionError(t)


def py_type(f):
    base = {"f32": "float", "f64": "float", "fixed": "float", "bool": "bool", "enum": f.enum if f.type == "enum" else "",
            "char": "str"}.get(f.type, "int")
    if f.type == "bytes_tail":
        return "str" if f.text else "bytes"
    if f.is_array and f.type != "char":
        return "list[%s]" % base
    return base


def gen_py_file(pf, runtime_import):
    L = []
    w = L.append
    w('"""GENERATED by tools/gen_packets.py from schema/%s.toml. DO NOT EDIT."""' % pf["stem"])
    w("import dataclasses")
    w("import enum")
    w("from typing import ClassVar, Optional")
    w("")
    w(runtime_import)
    w("")
    for en, e in pf["enums"].items():
        w("")
        w("class %s(enum.IntEnum):" % en)
        for k, v in e["values"].items():
            w("    %s = %d" % (k, v))
        w("")
    for pk in pf["packets"]:
        w("")
        w("@dataclasses.dataclass")
        w("class %s:" % pk["cls"])
        w('    """%s"""' % pk["doc"].replace('"""', "'''"))
        w("    PORT: ClassVar[Optional[int]] = %s" % ("None" if pk["port"] is None else "0x%02X" % pk["port"]))
        w("    IS_HEARTBEAT: ClassVar[bool] = %s" % pk["heartbeat"])
        w("    QOS: ClassVar[int] = %d  # %s" % (QOS[pk["qos"]], pk["qos"]))
        w("    LATEST: ClassVar[bool] = %s" % pk["latest"])
        w("    AIR_ELIGIBLE: ClassVar[bool] = %s" % pk["air"])
        w("    AIR_RATE_HZ: ClassVar[int] = %d" % pk["rate_air"])
        w("    USB_RATE_HZ: ClassVar[int] = %d" % pk["rate_usb"])
        w("    MAX_SIZE: ClassVar[int] = %d" % pk["max_size"])
        w("    MIN_SIZE: ClassVar[int] = %d" % pk["min_size"])
        for k, v in pk["consts"].items():
            w("    %s: ClassVar[int] = %d" % (k, v))
        for f in pk["fields"]:
            for b, v in f.bits.items():
                w("    %s_%s: ClassVar[int] = %d" % (f.name.upper(), b, 1 << v))
        for f in pk["fields"]:
            ty = py_type(f)
            if f.type == "bytes_tail":
                d = '""' if f.text else 'b""'
            elif f.type == "char":
                d = '""'
            elif f.is_array:
                d = "dataclasses.field(default_factory=lambda: [%s] * %d)" % (
                    {"float": "0.0", "bool": "False"}.get(ty[5:-1], "0") if f.type != "enum" else "0", f.n)
            else:
                d = {"float": "0.0", "bool": "False"}.get(ty, "0")  # enums: 0 (unknown values stay plain ints)
            c = ""
            if f.unit or f.doc:
                c = "  # " + (("[%s] " % f.unit) if f.unit else "") + f.doc.replace("\n", " ")
            w("    %s: %s = %s%s" % (f.name, ty, d, c))
        w("")
        tail = pk["fields"][-1] if pk["fields"] and pk["fields"][-1].type == "bytes_tail" else None
        w("    def encode(self) -> bytes:")
        w('        """Wire bytes. Raises ValueError on out-of-range integers or wrong array lengths."""')
        if tail:
            w("        tail = self.%s.encode('utf-8') if isinstance(self.%s, str) else bytes(self.%s)" % ((tail.name,) * 3) if tail.text else
              "        tail = bytes(self.%s)" % tail.name)
            w("        if len(tail) > %d:" % tail.max)
            w("            raise ValueError('%s too long')" % tail.name)
            w("        b = bytearray(%d + len(tail))" % tail.offset)
        else:
            w("        b = bytearray(%d)" % pk["max_size"])
        for f in pk["fields"]:
            c = py_valid_cond(f, "self." + f.name) if not f.is_array else None
            if c:
                w("        if not (%s):" % c)
                w("            raise ValueError('%s: value not allowed')" % f.name)
        for f in pk["fields"]:
            if f.type == "bytes_tail":
                w("        b[%d:] = tail" % f.offset)
            elif f.type == "char":
                w("        _rt.put_chars(b, %d, %d, self.%s)" % (f.offset, f.n, f.name))
            elif f.is_array:
                w("        if len(self.%s) != %d:" % (f.name, f.n))
                w("            raise ValueError('%s needs %d elements')" % (f.name, f.n))
                w("        for i, v in enumerate(self.%s):" % f.name)
                w("            " + py_put(f, "%d + i * %d" % (f.offset, f.size), "v"))
            else:
                w("        " + py_put(f, str(f.offset), "self." + f.name))
        w("        return bytes(b)")
        w("")
        w("    @classmethod")
        w("    def decode(cls, data) -> \"%s\":" % pk["cls"])
        w('        """Raises ValueError if shorter than MIN_SIZE. Longer input is accepted, extra bytes ignored."""')
        w("        data = bytes(data)")
        w("        if len(data) < cls.MIN_SIZE:")
        w("            raise ValueError('%s: payload too short (%%d < %d)' %% len(data))" % (pk["name"], pk["min_size"]))
        w("        t = cls()")
        for f in pk["fields"]:
            end = f.offset + f.total
            ind = "        "
            if f.type == "bytes_tail":
                w("        raw = data[%d:]" % f.offset)
                w("        if len(raw) > %d:" % f.max)
                w("            raise ValueError('%s too long')" % f.name)
                w("        t.%s = raw.decode('utf-8', errors='replace') if %s else raw" % (f.name, f.text))
                continue
            if f.optional:
                w("        if len(data) >= %d:" % end)
                ind = "            "
            if f.type == "char":
                w(ind + "t.%s = _rt.get_chars(data, %d, %d)" % (f.name, f.offset, f.n))
            elif f.is_array:
                w(ind + "t.%s = [%s for i in range(%d)]" % (f.name, py_get(f, "%d + i * %d" % (f.offset, f.size)), f.n))
            else:
                w(ind + "t.%s = %s" % (f.name, py_get(f, str(f.offset))))
                c = py_valid_cond(f, "t." + f.name)
                if c:
                    w(ind + "if not (%s):" % c)
                    w(ind + "    raise ValueError('%s: value not allowed')" % f.name)
            if f.optional:
                w("        else:")
                if f.is_array:
                    w("            t.%s = [%s] * %d" % (f.name, py_default(f), f.n))
                else:
                    w("            t.%s = %s" % (f.name, py_default(f)))
        w("        return t")
        w("")
    return "\n".join(L) + "\n"


def gen_py_registry(parsed, h):
    L = ['"""GENERATED by tools/gen_packets.py. DO NOT EDIT."""']
    for pf in parsed:
        L.append("from .%s import *  # noqa: F401,F403" % pf["stem"])
    allp = [pk for pf in parsed for pk in pf["packets"]]
    L.append("")
    L.append("from . import " + ", ".join(pf["stem"] for pf in parsed) + "  # noqa: E402")
    L.append("")
    L.append("#: FNV-1a-32 over the canonical serialisation of all schema files (carried in heartbeats).")
    L.append("SCHEMA_HASH = 0x%08X" % h)
    L.append("")
    L.append("#: port -> packet class")
    L.append("PACKETS_BY_PORT = {")
    for pf in parsed:
        for pk in pf["packets"]:
            if pk["port"] is not None:
                L.append("    0x%02X: %s.%s," % (pk["port"], pf["stem"], pk["cls"]))
    L.append("}")
    L.append("")
    L.append("#: schema name -> packet class")
    L.append("PACKETS_BY_NAME = {")
    for pf in parsed:
        for pk in pf["packets"]:
            L.append("    '%s': %s.%s," % (pk["name"], pf["stem"], pk["cls"]))
    L.append("}")
    L.append("")
    return "\n".join(L)


# ------------------------------------------------------------------ reference document

def field_type_text(f):
    t = f.type
    if t == "fixed":
        t = "fixed %s (scale %s, invalid %d)" % (f.wire, lit(f.scale), f.invalid)
    elif t == "enum":
        t = "enum %s" % f.enum
    elif t == "bytes_tail":
        t = "bytes_tail (max %d%s)" % (f.max, ", UTF-8 text" if f.text else "")
    elif t == "char":
        t = "char[%d]" % f.n
        return t
    if f.is_array:
        t += " x %d" % f.n
    return t


def gen_doc(parsed, h):
    L = ["<!-- GENERATED by tools/gen_packets.py from schema/*.toml. DO NOT EDIT. -->", "# Packet reference", "",
         "Schema hash: `0x%08X` (FNV-1a-32). Generated from the schema files; see [schema-format.md](schema-format.md)." % h,
         "All multi-byte values are little-endian. Sizes are payload bytes after the 2-byte transport header.", "",
         "## Port map", "",
         "| Port | Packet | Size (min-max) | QoS | Latest | Air Hz | USB Hz | Schema |", "|---|---|---|---|---|---|---|---|"]
    for pf in parsed:
        for pk in pf["packets"]:
            port = "-" if pk["port"] is None else "0x%02X" % pk["port"]
            size = str(pk["max_size"]) if pk["min_size"] == pk["max_size"] else "%d-%d" % (pk["min_size"], pk["max_size"])
            L.append("| %s | `%s` | %s | %s | %s | %s | %s | %s |" % (port, pk["name"], size, pk["qos"], "yes" if pk["latest"] else "no",
                                                                    pk["rate_air"] or "-", pk["rate_usb"] or "-", pf["stem"]))
    for pf in parsed:
        for en, e in pf["enums"].items():
            L += ["", "## Enum `%s` (%s)" % (en, e["type"]), ""]
            L.append(", ".join("`%s` = %d" % (k, v) for k, v in e["values"].items()))
        for pk in pf["packets"]:
            L += ["", "## %s%s" % (pk["name"], "" if pk["port"] is None else " (port 0x%02X)" % pk["port"]), "", pk["doc"], ""]
            if pk["consts"]:
                L += ["Constants: " + ", ".join("`%s` = %d" % kv for kv in pk["consts"].items()), ""]
            L += ["| Offset | Field | Type | Unit | Notes |", "|---|---|---|---|---|"]
            for f in pk["fields"]:
                note = f.doc
                if f.bits:
                    note += " Bits: " + ", ".join("%s = bit %d" % kv for kv in f.bits.items()) + "."
                if f.valid is not None or f.min_value is not None:
                    note += " Allowed:" + (" " + ", ".join(str(x) for x in f.valid) if f.valid is not None else "") + (
                        " (min %d)" % f.min_value if f.min_value is not None else "") + "; any other value rejects the whole frame."
                if f.optional:
                    note += " Optional (append-only), default %s." % ("NaN" if f.type in ("f32", "f64", "fixed") else f.default)
                L.append("| %d | `%s` | %s | %s | %s |" % (f.offset, f.name, field_type_text(f), f.unit, note.strip()))
    return "\n".join(L) + "\n"


# ------------------------------------------------------------------ driver

def generate(paths, ns, py_runtime):
    files, parsed = load(paths)
    h = schema_hash(files)
    cpp, py = {}, {}
    if py_runtime:
        mod, _, name = py_runtime.rpartition(".")
        rt_import = "from %s import %s as _rt" % (mod, name)
    else:
        rt_import = "from . import _runtime as _rt"
    for pf in parsed:
        cpp["ExVectrPackets/gen/%s.hpp" % pf["stem"]] = gen_cpp_file(pf, ns, "ExVectrPackets/gen/codec.hpp")
        py["%s.py" % pf["stem"]] = gen_py_file(pf, rt_import)
    cpp["ExVectrPackets/gen/%s" % ("all.hpp" if ns == "VCTR::Packets" else "all_" + parsed[0]["stem"] + ".hpp")] = gen_cpp_all(parsed, ns, h)
    py["__init__.py"] = gen_py_registry(parsed, h)
    return cpp, py, h


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--schema", action="append", required=True)
    ap.add_argument("--cpp-out", required=True)
    ap.add_argument("--py-out", required=True)
    ap.add_argument("--doc-out", default=None, help="write a markdown packet reference to this file")
    ap.add_argument("--cpp-namespace", default="VCTR::Packets")
    ap.add_argument("--py-runtime", default=None, help="module providing the runtime helpers (default: ._runtime)")
    ap.add_argument("--check", action="store_true", help="compare with files on disk, write nothing")
    a = ap.parse_args(argv)
    try:
        cpp, py, h = generate(a.schema, a.cpp_namespace, a.py_runtime)
    except (SchemaError, KeyError) as e:
        print("schema error: %s" % e, file=sys.stderr)
        return 2
    outputs = [(Path(a.cpp_out) / k, v) for k, v in cpp.items()] + [(Path(a.py_out) / k, v) for k, v in py.items()]
    if a.doc_out:
        outputs.append((Path(a.doc_out), gen_doc(load(a.schema)[1], h)))
    bad = 0
    for path, content in outputs:
        if a.check:
            if not path.exists() or path.read_text() != content:
                print("DIFFERS: %s" % path, file=sys.stderr)
                bad += 1
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
    if a.check:
        if bad:
            print("%d generated file(s) differ from the schema: run tools/regen.sh" % bad, file=sys.stderr)
            return 1
        print("generated files up to date (schema hash 0x%08X)" % h)
    else:
        print("wrote %d files (schema hash 0x%08X)" % (len(outputs), h))
    return 0


if __name__ == "__main__":
    sys.exit(main())
