"""Python side of the packet tests: shared vectors, drift check, fixed-point rules, schema validation.
Run from the repository root:  python3 -m unittest discover -s test/python -t .   (or: python3 -m unittest)"""
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "python"))
sys.path.insert(0, str(ROOT / "test" / "generated" / "py"))
sys.path.insert(0, str(ROOT / "tools"))

import exvectr_packets as pk  # noqa: E402
import exvectr_test_packets as tpk  # noqa: E402
from exvectr_packets import _runtime as rt  # noqa: E402
import gen_packets  # noqa: E402

VECTORS = json.loads((ROOT / "test" / "vectors" / "packets.json").read_text())
BY_NAME = {**pk.PACKETS_BY_NAME, **tpk.PACKETS_BY_NAME}


def conv(cur, v):
    """JSON value -> python value for a field whose default is `cur`."""
    if isinstance(cur, float):
        if v is None:
            return math.nan
        if isinstance(v, str):
            return float(v)
        return float(v)
    if isinstance(cur, bytes):
        return bytes.fromhex(v["hex"])
    if isinstance(cur, list):
        return [conv(cur[0], x) for x in v] if cur else v
    return v


def build(cls, values):
    obj = cls()
    for k, v in values.items():
        setattr(obj, k, conv(getattr(obj, k), v))
    return obj


def same(a, b, path=""):
    if isinstance(a, float) or isinstance(b, float):
        if isinstance(a, float) and isinstance(b, float) and math.isnan(a) and math.isnan(b):
            return
        assert isinstance(a, (int, float)) and isinstance(b, (int, float)), "%s: %r vs %r" % (path, a, b)
        assert math.isclose(a, b, rel_tol=1e-6, abs_tol=1e-12), "%s: %r != %r" % (path, a, b)
    elif isinstance(a, list):
        assert len(a) == len(b), path
        for i, (x, y) in enumerate(zip(a, b)):
            same(x, y, "%s[%d]" % (path, i))
    else:
        assert a == b, "%s: %r != %r" % (path, a, b)


def assert_obj(obj, expected_values):
    exp = build(type(obj), expected_values)
    for k in expected_values:
        same(getattr(obj, k), getattr(exp, k), k)


class VectorTests(unittest.TestCase):
    def test_encode_matches_vectors(self):
        for c in VECTORS["cases"]:
            with self.subTest(c["name"]):
                cls = BY_NAME[c["packet"]]
                self.assertEqual(build(cls, c["values"]).encode().hex(), c["hex"])
                self.assertLessEqual(len(bytes.fromhex(c["hex"])), cls.MAX_SIZE)

    def test_decode_matches_vectors(self):
        for c in VECTORS["cases"] + VECTORS["decode_only"]:
            with self.subTest(c["name"]):
                cls = BY_NAME[c["packet"]]
                obj = cls.decode(bytes.fromhex(c["hex"]))
                assert_obj(obj, c.get("decoded", c.get("values")))

    def test_decode_failures(self):
        for c in VECTORS["decode_fail"]:
            with self.subTest(c["name"]):
                with self.assertRaises(ValueError):
                    BY_NAME[c["packet"]].decode(bytes.fromhex(c["hex"]))

    def test_decoded_types_are_plain(self):
        obj = pk.PACKETS_BY_NAME["HANDSET_TELEM"].decode(bytes.fromhex(
            next(c["hex"] for c in VECTORS["cases"] if c["name"] == "handset_telem")))
        self.assertIs(type(obj.pitch), float)
        self.assertIs(type(obj.satellites), int)
        self.assertIs(type(obj.mode_text), str)
        ack = pk.RequestAck.decode(bytes.fromhex("02010400"))
        self.assertIs(ack.status, pk.RequestStatus.FAILED)
        self.assertEqual(pk.RequestAck.decode(bytes.fromhex("020163 00".replace(" ", ""))).status, 0x63)  # unknown stays int


class FixedRules(unittest.TestCase):
    def test_round_half_away(self):
        for x, r in [(0.5, 1), (-0.5, -1), (1.5, 2), (2.5, 3), (-2.5, -3), (0.49999999999999994, 0), (-0.49999999999999994, 0),
                     (2.4999999999999996, 2), (0.0, 0)]:
            self.assertEqual(rt.round_half_away(x), r, x)

    def test_fixed_raw(self):
        f = lambda v: rt.fixed_raw(v, 1.0, -32767, 32767, -32768)  # noqa: E731
        self.assertEqual(f(math.nan), -32768)
        self.assertEqual(f(math.inf), -32768)
        self.assertEqual(f(-math.inf), -32768)
        self.assertEqual(f(1e300), 32767)
        self.assertEqual(f(-1e300), -32767)
        self.assertEqual(f(-0.0), 0)

    def test_decode_invalid_is_nan(self):
        buf = bytearray(2)
        buf[:] = (0x8000).to_bytes(2, "little")
        self.assertTrue(math.isnan(rt.get_fixed(bytes(buf), "<h", 0, 0.5, -32768)))

    def test_roundtrip_error_within_half_lsb(self):
        scale = 1e-4
        for i in range(-3000, 3000):
            v = i * 1.2345e-3
            raw = rt.fixed_raw(v, scale, -32767, 32767, -32768)
            if abs(raw) < 32767:
                self.assertLessEqual(abs(raw * scale - v), scale / 2 + 1e-12)

    def test_encode_errors(self):
        with self.assertRaises(ValueError):
            tpk.KitchenSink(u8v=256).encode()
        with self.assertRaises(ValueError):
            pk.RcState(ch=[0] * 15).encode()
        with self.assertRaises(ValueError):
            pk.HandsetTelem(mode_text="123456789").encode()
        with self.assertRaises(ValueError):
            pk.LogText(text="x" * 234).encode()
        self.assertEqual(len(pk.LogText(text="x" * 233).encode()), 234)

    def test_nan_and_overflow_floats(self):
        self.assertEqual(pk.Request(arg_f=math.nan).encode()[8:], bytes.fromhex("0000c07f"))
        self.assertEqual(pk.Request(arg_f=1e39).encode()[8:], bytes.fromhex("0000807f"))
        self.assertEqual(pk.Request(arg_f=-1e39).encode()[8:], bytes.fromhex("000080ff"))


class SchemaTests(unittest.TestCase):
    def test_generated_files_up_to_date(self):
        r = subprocess.run([sys.executable, str(ROOT / "tools" / "gen_packets.py"), "--check",
                            "--schema", str(ROOT / "schema/common.toml"), "--schema", str(ROOT / "schema/starship.toml"),
                            "--cpp-out", str(ROOT / "gen"), "--py-out", str(ROOT / "python/exvectr_packets"),
                            "--doc-out", str(ROOT / "docs/packet-reference.md")],
                           capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)
        r = subprocess.run([sys.executable, str(ROOT / "tools" / "gen_packets.py"), "--check",
                            "--schema", str(ROOT / "test/schema/test_packets.toml"), "--cpp-namespace", "VCTR::PacketsTest",
                            "--py-runtime", "exvectr_packets._runtime", "--cpp-out", str(ROOT / "test/generated/cpp"),
                            "--py-out", str(ROOT / "test/generated/py/exvectr_test_packets")], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)

    def test_vectors_file_up_to_date(self):
        r = subprocess.run([sys.executable, str(ROOT / "tools" / "make_vectors.py"), "--check"])
        self.assertEqual(r.returncode, 0, "run tools/make_vectors.py")

    def test_drift_is_detected(self):
        with tempfile.TemporaryDirectory() as d:
            args = ["--schema", str(ROOT / "schema/common.toml"), "--cpp-out", d + "/cpp", "--py-out", d + "/py"]
            gen = str(ROOT / "tools" / "gen_packets.py")
            self.assertEqual(subprocess.run([sys.executable, gen] + args, capture_output=True).returncode, 0)
            self.assertEqual(subprocess.run([sys.executable, gen, "--check"] + args, capture_output=True).returncode, 0)
            f = Path(d) / "py" / "common.py"
            f.write_text(f.read_text() + "# edited\n")
            self.assertEqual(subprocess.run([sys.executable, gen, "--check"] + args, capture_output=True).returncode, 1)

    def test_registry(self):
        self.assertEqual(len(pk.PACKETS_BY_PORT), len([c for c in pk.PACKETS_BY_NAME.values() if c.PORT is not None]))
        for port, cls in pk.PACKETS_BY_PORT.items():
            self.assertEqual(cls.PORT, port)
            self.assertLessEqual(cls.MAX_SIZE, 234)
            self.assertEqual(cls.AIR_ELIGIBLE, port < 0x80)
            if not cls.AIR_ELIGIBLE:
                self.assertEqual(cls.AIR_RATE_HZ, 0)

    def test_air_set_budget(self):
        """Radio set: payload + 7 bytes per frame, below 35 % of the 6150 B/s air downlink entry capacity (the limit
        was 1.2 kB/s, design 3.5.6, before AIR_ACTUATORS and AIR_MISSION)."""
        total = sum((c.MAX_SIZE + 7) * c.AIR_RATE_HZ for c in pk.PACKETS_BY_NAME.values())
        self.assertLess(total, 2150)

    def test_hash_rules(self):
        base = {"common": {"packet": {"A": {"port": 1, "fields": [{"name": "x", "type": "u8", "doc": "one"}]}}}}
        h = gen_packets.schema_hash(base)
        same_doc = json.loads(json.dumps(base))
        same_doc["common"]["packet"]["A"]["fields"][0]["doc"] = "changed text"
        self.assertEqual(gen_packets.schema_hash(same_doc), h)
        diff = json.loads(json.dumps(base))
        diff["common"]["packet"]["A"]["fields"][0]["type"] = "i8"
        self.assertNotEqual(gen_packets.schema_hash(diff), h)
        self.assertEqual(gen_packets.fnv1a32(b"123456789"), 0xBB86B11C)  # FNV-1a-32 check value (0xBB86B11C)
        self.assertEqual(pk.SCHEMA_HASH, gen_packets.schema_hash(gen_packets.load(
            [ROOT / "schema/common.toml", ROOT / "schema/starship.toml"])[0]))

    def _expect_error(self, toml_text):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "bad.toml"
            p.write_text(toml_text)
            with self.assertRaises((gen_packets.SchemaError, KeyError)):
                gen_packets.load([p])

    def test_schema_validation(self):
        head = '[packet.A]\nport = 0x20\nqos = "link"\nlatest = true\n'
        self._expect_error(head + 'fields = [{ name = "x", type = "u8" }, { name = "x", type = "u8" }]')  # duplicate
        self._expect_error(head + 'fields = [{ name = "x", type = "u8", count = 300 }]')  # too big
        self._expect_error(head + 'fields = [{ name = "t", type = "bytes_tail", max = 4 }, { name = "x", type = "u8" }]')
        self._expect_error(head + 'fields = [{ name = "x", type = "u8", optional = true }, { name = "y", type = "u8" }]')
        self._expect_error(head + 'fields = [{ name = "x", type = "fixed", wire = "i16" }]')  # no scale/limit
        self._expect_error(head + 'fields = [{ name = "x", type = "fixed", wire = "u16", limit = 1.0 }]')
        self._expect_error(head + 'fields = [{ name = "x", type = "nope" }]')
        self._expect_error(head.replace("0x20", "0x80") + 'rate_air = 1\nfields = [{ name = "x", type = "u8" }]')
        self._expect_error(head + 'fields = [{ name = "x", type = "u8" }]\n[packet.B]\nport = 0x20\nqos = "link"\nlatest = true\nfields = []')


if __name__ == "__main__":
    unittest.main()
