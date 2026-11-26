// Host tests: the generated C++ must reproduce test/vectors/packets.json byte for byte (shared with the Python
// tests), follow the fixed-point rules, and match the committed files after regeneration.
#include <unity.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include "ExVectrPackets/gen/all.hpp"
#include "ExVectrPackets/gen/all_test_packets.hpp"

using namespace VCTR::Packets;
using namespace VCTR::PacketsTest;
using Bytes = std::vector<uint8_t>;

void setUp() {}
void tearDown() {}

static void fail(const std::string& m) { TEST_FAIL_MESSAGE(m.c_str()); }

// ---------------------------------------------------------------- minimal JSON reader (test code, heap allowed)
struct J {
    enum Kind { Null, Bool, Num, Str, Arr, Obj } kind = Null;
    bool b = false;
    std::string s;  // string value, or the raw number token
    std::vector<J> a;
    std::map<std::string, J> o;
    const J* get(const std::string& k) const {
        auto it = o.find(k);
        return it == o.end() ? nullptr : &it->second;
    }
};

struct Parser {
    const std::string& t;
    size_t i = 0;
    explicit Parser(const std::string& text) : t(text) {}
    void ws() { while (i < t.size() && isspace((unsigned char)t[i])) ++i; }
    J value() {
        ws();
        J j;
        char c = t.at(i);
        if (c == '{') {
            j.kind = J::Obj; ++i; ws();
            if (t[i] == '}') { ++i; return j; }
            for (;;) {
                ws(); std::string k = str(); ws(); ++i;  // ':'
                j.o[k] = value(); ws();
                if (t[i++] == '}') break;
            }
        } else if (c == '[') {
            j.kind = J::Arr; ++i; ws();
            if (t[i] == ']') { ++i; return j; }
            for (;;) {
                j.a.push_back(value()); ws();
                if (t[i++] == ']') break;
            }
        } else if (c == '"') {
            j.kind = J::Str; j.s = str();
        } else if (t.compare(i, 4, "null") == 0) {
            i += 4;
        } else if (t.compare(i, 4, "true") == 0) {
            j.kind = J::Bool; j.b = true; i += 4;
        } else if (t.compare(i, 5, "false") == 0) {
            j.kind = J::Bool; i += 5;
        } else {
            j.kind = J::Num;
            size_t st = i;
            while (i < t.size() && (isdigit((unsigned char)t[i]) || strchr("+-.eE", t[i]))) ++i;
            j.s = t.substr(st, i - st);
        }
        return j;
    }
    std::string str() {  // handles \" \\ \/ \n \t \r \b \f \uXXXX (BMP) escapes
        std::string r;
        ++i;
        while (t[i] != '"') {
            if (t[i] == '\\') {
                ++i;
                char e = t[i++];
                if (e == 'u') {
                    unsigned cp = (unsigned)strtoul(t.substr(i, 4).c_str(), nullptr, 16);
                    i += 4;
                    if (cp < 0x80) r += (char)cp;
                    else if (cp < 0x800) { r += (char)(0xC0 | (cp >> 6)); r += (char)(0x80 | (cp & 0x3F)); }
                    else { r += (char)(0xE0 | (cp >> 12)); r += (char)(0x80 | ((cp >> 6) & 0x3F)); r += (char)(0x80 | (cp & 0x3F)); }
                } else {
                    r += (e == 'n') ? '\n' : (e == 't') ? '\t' : (e == 'r') ? '\r' : (e == 'b') ? '\b' : (e == 'f') ? '\f' : e;
                }
            } else {
                r += t[i++];
            }
        }
        ++i;
        return r;
    }
};

static J loadVectors() {
    const char* paths[] = {"test/vectors/packets.json", "../test/vectors/packets.json", "../../test/vectors/packets.json"};
    for (const char* p : paths) {
        std::ifstream f(p);
        if (f) {
            std::stringstream ss;
            ss << f.rdbuf();
            std::string text = ss.str();
            Parser ps(text);
            return ps.value();
        }
    }
    fail("cannot open test/vectors/packets.json");
    return J();
}

static Bytes fromHex(const std::string& h) {
    Bytes b;
    for (size_t i = 0; i + 1 < h.size(); i += 2) b.push_back((uint8_t)strtoul(h.substr(i, 2).c_str(), nullptr, 16));
    return b;
}
static std::string toHex(const uint8_t* p, size_t n) {
    std::string s;
    char buf[3];
    for (size_t i = 0; i < n; ++i) { snprintf(buf, sizeof buf, "%02x", p[i]); s += buf; }
    return s;
}

// ---------------------------------------------------------------- visitors over generated packets
struct Assign {
    const J& v;
    std::string ctx;
    template <class T> static void put(T& dst, const J& j) {
        if constexpr (std::is_same_v<T, bool>) {
            dst = j.b;
        } else if constexpr (std::is_enum_v<T>) {
            dst = static_cast<T>(strtoll(j.s.c_str(), nullptr, 10));
        } else if constexpr (std::is_floating_point_v<T>) {
            if (j.kind == J::Null) dst = std::numeric_limits<T>::quiet_NaN();
            else if (j.kind == J::Str) dst = (T)strtod(j.s.c_str(), nullptr);  // "inf", "-inf"
            else dst = (T)strtod(j.s.c_str(), nullptr);
        } else if constexpr (std::is_signed_v<T>) {
            dst = (T)strtoll(j.s.c_str(), nullptr, 10);
        } else {
            dst = (T)strtoull(j.s.c_str(), nullptr, 10);
        }
    }
    template <class T> void scalar(const char* n, T& r) { if (auto* j = v.get(n)) put(r, *j); }
    template <class T, size_t N> void array(const char* n, T (&r)[N]) {
        if (auto* j = v.get(n)) for (size_t i = 0; i < N && i < j->a.size(); ++i) put(r[i], j->a[i]);
    }
    template <size_t N> void chars(const char* n, char (&r)[N]) {
        if (auto* j = v.get(n)) { memset(r, 0, N); memcpy(r, j->s.data(), j->s.size() < N ? j->s.size() : N); }
    }
    template <size_t N> void tail(const char* n, uint8_t (&r)[N], uint16_t& len) {
        auto* j = v.get(n);
        if (!j) return;
        Bytes b = (j->kind == J::Obj) ? fromHex(j->get("hex")->s) : Bytes(j->s.begin(), j->s.end());
        memcpy(r, b.data(), b.size());
        len = (uint16_t)b.size();
    }
};

struct Check {
    const J& v;
    std::string ctx;
    template <class T> static std::string cmp(const T& got, const J& j) {
        char buf[160];
        if constexpr (std::is_same_v<T, bool>) {
            if (got != j.b) return "bool mismatch";
        } else if constexpr (std::is_enum_v<T>) {
            if ((long long)got != strtoll(j.s.c_str(), nullptr, 10)) return "enum mismatch";
        } else if constexpr (std::is_floating_point_v<T>) {
            double exp = (j.kind == J::Null) ? NAN : strtod(j.s.c_str(), nullptr);
            if (std::isnan(exp)) {
                if (!std::isnan(got)) { snprintf(buf, sizeof buf, "expected NaN, got %.9g", (double)got); return buf; }
            } else if (!(fabs((double)got - exp) <= 1e-6 * fabs(exp) + 1e-12)) {
                snprintf(buf, sizeof buf, "expected %.12g, got %.12g", exp, (double)got);
                return buf;
            }
        } else if constexpr (std::is_signed_v<T>) {
            if ((long long)got != strtoll(j.s.c_str(), nullptr, 10)) return "int mismatch";
        } else {
            if ((unsigned long long)got != strtoull(j.s.c_str(), nullptr, 10)) return "uint mismatch";
        }
        return "";
    }
    template <class T> void scalar(const char* n, T& r) {
        auto* j = v.get(n);
        if (!j) return;
        std::string e = cmp(r, *j);
        if (!e.empty()) fail(ctx + "." + n + ": " + e);
    }
    template <class T, size_t N> void array(const char* n, T (&r)[N]) {
        auto* j = v.get(n);
        if (!j) return;
        if (j->a.size() != N) fail(ctx + "." + n + ": array length");
        for (size_t i = 0; i < N; ++i) {
            std::string e = cmp(r[i], j->a[i]);
            if (!e.empty()) fail(ctx + "." + n + "[" + std::to_string(i) + "]: " + e);
        }
    }
    template <size_t N> void chars(const char* n, char (&r)[N]) {
        auto* j = v.get(n);
        if (!j) return;
        if (std::string(r, strnlen(r, N)) != j->s) fail(ctx + "." + n + ": text mismatch");
    }
    template <size_t N> void tail(const char* n, uint8_t (&r)[N], uint16_t& len) {
        auto* j = v.get(n);
        if (!j) return;
        Bytes b = (j->kind == J::Obj) ? fromHex(j->get("hex")->s) : Bytes(j->s.begin(), j->s.end());
        if (b.size() != len || memcmp(b.data(), r, len) != 0) fail(ctx + "." + n + ": tail mismatch");
    }
};

// ---------------------------------------------------------------- per-type operations
template <class T>
struct Shape {  // the interface PacketPort<T> (ExVectrNetwork) needs
    static_assert(T::kMaxSize <= 234, "payload too large");
    static_assert(std::is_same_v<decltype(std::declval<const T&>().encode((uint8_t*)nullptr, size_t(0))), size_t>, "encode");
    static_assert(std::is_same_v<decltype(std::declval<T&>().decode((const uint8_t*)nullptr, size_t(0))), bool>, "decode");
    static_assert(std::is_default_constructible_v<T> && std::is_copy_assignable_v<T>, "value type");
    static constexpr bool ok = true;
};

template <class T>
void opEncode(const J& c, const std::string& name) {
    static_assert(Shape<T>::ok, "");
    T p;
    Assign a{*c.get("values"), name};
    p.visit(a);
    Bytes exp = fromHex(c.get("hex")->s);
    TEST_ASSERT_TRUE_MESSAGE(exp.size() <= T::kMaxSize, name.c_str());
    uint8_t buf[T::kMaxSize + 4];
    memset(buf, 0xEE, sizeof buf);
    size_t n = p.encode(buf, sizeof buf);
    if (toHex(buf, n) != c.get("hex")->s) fail(name + ": encode " + toHex(buf, n) + " != " + c.get("hex")->s);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0xEE, buf[n], "wrote past the end");
    // exact capacity works, one byte less fails, null fails
    uint8_t exact[T::kMaxSize + 1];
    TEST_ASSERT_EQUAL_MESSAGE(n, p.encode(exact, n), name.c_str());
    TEST_ASSERT_EQUAL_MESSAGE(0, p.encode(exact, n - 1), name.c_str());
    TEST_ASSERT_EQUAL_MESSAGE(0, p.encode(nullptr, sizeof buf), name.c_str());
}

template <class T>
void opDecode(const J& c, const std::string& name) {
    Bytes in = fromHex(c.get("hex")->s);
    T p;
    if (!p.decode(in.data(), in.size())) fail(name + ": decode failed");
    const J* exp = c.get("decoded");
    if (!exp) exp = c.get("values");
    Check ch{*exp, name};
    p.visit(ch);
}

struct Ops {
    const char* name;
    void (*enc)(const J&, const std::string&);
    void (*dec)(const J&, const std::string&);
    bool (*decOk)(const uint8_t*, size_t);
};
template <class T> bool tryDecode(const uint8_t* p, size_t n) { T t; return t.decode(p, n); }
#define OPS(N, T) {N, &opEncode<T>, &opDecode<T>, &tryDecode<T>}
static const Ops kOps[] = {
    OPS("HEARTBEAT", Heartbeat), OPS("LINK_STATS", LinkStats), OPS("LOG_TEXT", LogText), OPS("LINK_CONFIG", LinkConfig),
    OPS("HANDSET_TELEM", HandsetTelem), OPS("RC_STATE", RcState), OPS("REQUEST", Request),
    OPS("REQUEST_ACK", RequestAck), OPS("VEHICLE_STATUS", VehicleStatus), OPS("AIR_POSE", AirPose), OPS("AIR_ACTUATORS", AirActuators), OPS("AIR_MISSION", AirMission), OPS("AIR_GNSS", AirGnss), OPS("AIR_GNSS_INFO", AirGnssInfo), OPS("HOME_POSITION", HomePosition), OPS("MISSION_PARAMS", MissionParams),
    OPS("POSE", Pose), OPS("ACTUATORS", Actuators), OPS("MISSION", Mission), OPS("GNSS", Gnss), OPS("CMD_APPLIED", CmdApplied), OPS("CMD_STATE", CmdState),
    OPS("IMU_RAW", ImuRaw), OPS("ESTIMATOR", Estimator), OPS("TASK_TIMING", TaskTiming), OPS("CALIBRATION", Calibration),
    OPS("KITCHEN_SINK", KitchenSink), OPS("OPTIONAL_TEST", OptionalTest),
};
static const Ops& find(const std::string& n) {
    for (const Ops& o : kOps) if (n == o.name) return o;
    fail("unknown packet " + n);
    return kOps[0];
}

// ---------------------------------------------------------------- tests
static void test_vectors_encode() {
    J v = loadVectors();
    TEST_ASSERT_TRUE(v.get("cases")->a.size() > 30);
    for (const J& c : v.get("cases")->a) find(c.get("packet")->s).enc(c, c.get("name")->s);
}
static void test_vectors_decode() {
    J v = loadVectors();
    for (const J& c : v.get("cases")->a) find(c.get("packet")->s).dec(c, c.get("name")->s);
    for (const J& c : v.get("decode_only")->a) find(c.get("packet")->s).dec(c, c.get("name")->s);
}
static void test_vectors_decode_failures() {
    J v = loadVectors();
    TEST_ASSERT_TRUE(v.get("decode_fail")->a.size() >= 5);
    for (const J& c : v.get("decode_fail")->a) {
        Bytes in = fromHex(c.get("hex")->s);
        if (find(c.get("packet")->s).decOk(in.data(), in.size())) fail(c.get("name")->s + ": decode should fail");
    }
}

static void test_decode_failure_leaves_object_untouched() {
    RcState p;
    p.flags = 9;
    uint8_t shortBuf[10] = {0};
    TEST_ASSERT_FALSE(p.decode(shortBuf, sizeof shortBuf));
    TEST_ASSERT_EQUAL_UINT8(9, p.flags);
    TEST_ASSERT_FALSE(p.decode(nullptr, 34));
}

static void test_cmd_state_strict() {
    CmdState p;
    p.flags = 1;
    p.version = 1;
    p.session = 7;
    uint8_t buf[14];
    TEST_ASSERT_EQUAL_UINT32(14, p.encode(buf, sizeof buf));
    CmdState q;
    q.session = 99;
    TEST_ASSERT_TRUE(q.decode(buf, sizeof buf));
    TEST_ASSERT_EQUAL_UINT32(7, q.session);
    buf[6] = 2;  // go out of range: whole frame rejected, object untouched
    q.session = 99;
    TEST_ASSERT_FALSE(q.decode(buf, sizeof buf));
    TEST_ASSERT_EQUAL_UINT32(99, q.session);
    p.session = 0;  // not allowed on encode either
    TEST_ASSERT_EQUAL_UINT32(0, p.encode(buf, sizeof buf));
    Request r;
    uint8_t old[12] = {0};
    r.session = 5;
    TEST_ASSERT_TRUE(r.decode(old, sizeof old));  // older sender without session
    TEST_ASSERT_EQUAL_UINT32(0, r.session);
}

static void test_fixed_rules_direct() {
    using namespace VCTR::Packets::codec;
    TEST_ASSERT_EQUAL_INT64(3, fixedEncode(2.5, 1.0, -32767, 32767, -32768));
    TEST_ASSERT_EQUAL_INT64(-3, fixedEncode(-2.5, 1.0, -32767, 32767, -32768));
    TEST_ASSERT_EQUAL_INT64(0, fixedEncode(0.49999999999999994, 1.0, -32767, 32767, -32768));
    TEST_ASSERT_EQUAL_INT64(-32768, fixedEncode(NAN, 1.0, -32767, 32767, -32768));
    TEST_ASSERT_EQUAL_INT64(-32768, fixedEncode(INFINITY, 1.0, -32767, 32767, -32768));
    TEST_ASSERT_EQUAL_INT64(-32768, fixedEncode(-INFINITY, 1.0, -32767, 32767, -32768));
    TEST_ASSERT_EQUAL_INT64(32767, fixedEncode(1e300, 1.0, -32767, 32767, -32768));
    TEST_ASSERT_EQUAL_INT64(-32767, fixedEncode(-1e300, 1.0, -32767, 32767, -32768));
    TEST_ASSERT_TRUE(std::isnan(fixedDecode(-32768, 1.0, -32768)));
    TEST_ASSERT_TRUE(fixedDecode(-32767, 1.0, -32768) == -32767.0);
    // generated scale constants: limit-based scale hits the limit exactly at full scale
    AirPose ap;
    ap.q[0] = 1.0f;
    ap.q[1] = -1.0f;
    ap.q[2] = NAN;
    ap.q[3] = 0.5f;
    uint8_t buf[AirPose::kMaxSize];
    TEST_ASSERT_EQUAL(AirPose::kMaxSize, ap.encode(buf, sizeof buf));
    AirPose back;
    TEST_ASSERT_TRUE(back.decode(buf, sizeof buf));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, back.q[0]);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, back.q[1]);
    TEST_ASSERT_TRUE(std::isnan(back.q[2]));
    TEST_ASSERT_FLOAT_WITHIN(2e-5f, 0.5f, back.q[3]);
}

static void test_air_pose_position_mm() {
    // pos is i32 with 1 mm resolution (was i16, 0.1 m); 32 byte payload (ground_distance u16 appended 2026-11-26)
    TEST_ASSERT_EQUAL(32, AirPose::kMaxSize);
    AirPose ap;
    ap.pos[0] = 12.345;
    ap.pos[1] = -4000.0; // beyond the old i16 0.1 m range (+-3276.6 m)
    ap.pos[2] = 1.2344;  // rounds to 1.234
    uint8_t buf[AirPose::kMaxSize];
    TEST_ASSERT_EQUAL(AirPose::kMaxSize, ap.encode(buf, sizeof buf));
    AirPose back;
    TEST_ASSERT_TRUE(back.decode(buf, sizeof buf));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 12.345f, back.pos[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -4000.0f, back.pos[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.234f, back.pos[2]);
}

static void test_metadata_constants() {
    TEST_ASSERT_EQUAL_UINT8(0x10, RcState::kPort);
    TEST_ASSERT_EQUAL_UINT8(0, RcState::kQos);
    TEST_ASSERT_TRUE(RcState::kLatest);
    TEST_ASSERT_EQUAL(34, RcState::kMaxSize);
    TEST_ASSERT_EQUAL(1, RcState::kFlagsSourceOk);
    TEST_ASSERT_EQUAL_INT32(-32768, RcState::kChInvalid);
    TEST_ASSERT_TRUE(AirPose::kAirEligible);
    TEST_ASSERT_EQUAL(0x40, VehicleStatus::kFlagsUartPeerSeen);
    TEST_ASSERT_FALSE(Pose::kAirEligible);
    TEST_ASSERT_EQUAL_UINT16(0, Pose::kAirRateHz);
    TEST_ASSERT_EQUAL_UINT16(100, Pose::kUsbRateHz);
    TEST_ASSERT_EQUAL(234, LogText::kMaxSize);
    TEST_ASSERT_EQUAL(11, Heartbeat::kMaxSize);
    TEST_ASSERT_TRUE(Heartbeat::kIsHeartbeat);
    TEST_ASSERT_NOT_EQUAL(0, VCTR::Packets::kSchemaHash);
    TEST_ASSERT_EQUAL(0x80, VehicleStatus::kFlagsZeroing);
    TEST_ASSERT_TRUE(AirActuators::kAirEligible);
    TEST_ASSERT_EQUAL(38, AirActuators::kMaxSize);
    TEST_ASSERT_EQUAL_UINT16(25, AirActuators::kAirRateHz);
    TEST_ASSERT_EQUAL_UINT16(0, AirActuators::kUsbRateHz);
    TEST_ASSERT_TRUE(AirMission::kAirEligible);
    TEST_ASSERT_EQUAL(46, AirMission::kMaxSize);
    TEST_ASSERT_EQUAL(30, AirMission::kMinSize);
    TEST_ASSERT_EQUAL_UINT16(5, AirMission::kAirRateHz);
    TEST_ASSERT_TRUE(AirGnss::kAirEligible);
    TEST_ASSERT_EQUAL(0x24, AirGnss::kPort);
    TEST_ASSERT_EQUAL(22, AirGnss::kMaxSize);
    TEST_ASSERT_EQUAL_UINT16(10, AirGnss::kAirRateHz);
    TEST_ASSERT_EQUAL(0x25, AirGnssInfo::kPort);
    TEST_ASSERT_EQUAL(10, AirGnssInfo::kMaxSize);
    TEST_ASSERT_EQUAL_UINT16(1, AirGnssInfo::kAirRateHz);
    TEST_ASSERT_EQUAL(38, VehicleStatus::kMaxSize);
    TEST_ASSERT_EQUAL(28, VehicleStatus::kMinSize);
    TEST_ASSERT_EQUAL_UINT16(10, Gnss::kUsbRateHz);
    TEST_ASSERT_EQUAL(0x50, HomePosition::kPort);
    TEST_ASSERT_EQUAL(25, HomePosition::kMaxSize);
    // every air-eligible packet fits the radio budget: < 3100 B/s including 7 bytes overhead, i.e. below 50.4 % of the
    // 6150 B/s air downlink entry capacity (was 1.2 kB/s, design 3.5.6, before AIR_ACTUATORS and AIR_MISSION; 2150
    // before AIR_GNSS (290 B/s) and AIR_GNSS_INFO (17 B/s); 2500 before AIR_POSE went from 10 to 25 Hz,
    // +555 B/s, 2026-11-22; 2979 B/s before AIR_POSE ground_distance, +50 B/s on 2026-11-26: the set is 3029 B/s now, limit raised to 3100)
    unsigned total = (VehicleStatus::kMaxSize + 7) * VehicleStatus::kAirRateHz + (AirPose::kMaxSize + 7) * AirPose::kAirRateHz +
                     (AirActuators::kMaxSize + 7) * AirActuators::kAirRateHz + (AirMission::kMaxSize + 7) * AirMission::kAirRateHz + (AirGnss::kMaxSize + 7) * AirGnss::kAirRateHz + (AirGnssInfo::kMaxSize + 7) * AirGnssInfo::kAirRateHz +
                     (HandsetTelem::kMaxSize + 7) * HandsetTelem::kAirRateHz + (LinkStats::kMaxSize + 7) * LinkStats::kAirRateHz;
    TEST_ASSERT_TRUE(total < 3100);
}

static void test_log_text_helper() {
    LogText l;
    l.severity = Severity::kWarning;
    l.set_text("hello");
    uint8_t buf[LogText::kMaxSize];
    TEST_ASSERT_EQUAL(6, l.encode(buf, sizeof buf));
    std::string longText(500, 'x');
    l.set_text(longText.c_str());
    TEST_ASSERT_EQUAL(233, l.text_len);
    TEST_ASSERT_EQUAL(234, l.encode(buf, sizeof buf));
    l.text_len = 300;  // corrupted length is refused, never over-read
    TEST_ASSERT_EQUAL(0, l.encode(buf, sizeof buf));
}

static void test_decode_never_overreads_fuzz() {
    // Random buffers of random length: decode must never crash (run under ASan/UBSan by the native-san env when
    // available) and a successful decode must re-encode within kMaxSize.
    uint32_t s = 12345;
    auto rnd = [&]() { s = s * 1664525u + 1013904223u; return s >> 8; };
    for (int it = 0; it < 20000; ++it) {
        size_t len = rnd() % 240;
        std::vector<uint8_t> in(len ? len : 1);
        for (auto& b : in) b = (uint8_t)rnd();
        for (const Ops& o : kOps) {
            (void)o.decOk(in.data(), len);
        }
        VehicleStatus vs;
        if (vs.decode(in.data(), len)) {
            uint8_t out[VehicleStatus::kMaxSize];
            TEST_ASSERT_EQUAL(VehicleStatus::kMaxSize, vs.encode(out, sizeof out));
        }
    }
}

static void test_generated_files_up_to_date() {
    if (system("python3 --version >/dev/null 2>&1") != 0) TEST_IGNORE_MESSAGE("python3 not available");
    const char* roots[] = {".", "..", "../.."};
    for (const char* r : roots) {
        std::string base = std::string(r) + "/";
        std::ifstream f(base + "tools/gen_packets.py");
        if (!f) continue;
        std::string cmd = "python3 " + base + "tools/gen_packets.py --check --schema " + base + "schema/common.toml --schema " + base +
                          "schema/starship.toml --cpp-out " + base + "gen --py-out " + base + "python/exvectr_packets --doc-out " + base + "docs/packet-reference.md";
        TEST_ASSERT_EQUAL_MESSAGE(0, system(cmd.c_str()), "generated production files differ from the schema");
        cmd = "python3 " + base + "tools/gen_packets.py --check --schema " + base + "test/schema/test_packets.toml --cpp-namespace VCTR::PacketsTest " +
              "--py-runtime exvectr_packets._runtime --cpp-out " + base + "test/generated/cpp --py-out " + base + "test/generated/py/exvectr_test_packets";
        TEST_ASSERT_EQUAL_MESSAGE(0, system(cmd.c_str()), "generated test files differ from the schema");
        return;
    }
    TEST_IGNORE_MESSAGE("repository root not found from the test working directory");
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_air_pose_position_mm);
    RUN_TEST(test_vectors_encode);
    RUN_TEST(test_vectors_decode);
    RUN_TEST(test_vectors_decode_failures);
    RUN_TEST(test_decode_failure_leaves_object_untouched);
    RUN_TEST(test_cmd_state_strict);
    RUN_TEST(test_fixed_rules_direct);
    RUN_TEST(test_metadata_constants);
    RUN_TEST(test_log_text_helper);
    RUN_TEST(test_decode_never_overreads_fuzz);
    RUN_TEST(test_generated_files_up_to_date);
    return UNITY_END();
}
