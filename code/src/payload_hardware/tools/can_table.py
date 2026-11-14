"""
can_table.py -- an INDEPENDENT Python implementation of the CAN data layout.

Used for (a) cross-checking the C++ codec with shared test vectors, (b) the fake payload
board for vcan testing, (c) bench tools.  It deliberately works on strings of '0'/'1'
characters instead of shifts and masks so that it cannot share a bug with the C++ code.

Layout:  [dest:1][msg id:6][payload:57], MSB first; a field "bit a, width w" has its most
significant bit at payload position a.  (Assumption A3 in docs/CAN_TABLE_REPORT.md)
"""
import json, os

HERE = os.path.dirname(os.path.abspath(__file__))
TABLE = json.load(open(os.path.join(HERE, "..", "config", "can_table.json")))
BY_NAME = {m["name"]: (int(i), m) for i, m in TABLE["messages"].items()}
BY_ID = {int(i): m for i, m in TABLE["messages"].items()}
ARB_ID = TABLE["frame"]["arbitration_id"]


def encode(name, dest, raw_fields):
    """raw_fields: {signal_name: integer raw value}. Returns the 64-bit data word as an int."""
    mid, m = BY_NAME[name]
    bits = ["0"] * 64
    bits[0] = str(dest)
    for k, ch in enumerate(format(mid, "06b")):
        bits[1 + k] = ch
    for s in m["signals"]:
        if s["name"] not in raw_fields:
            continue
        v = raw_fields[s["name"]]
        w = s["width"]
        if v < 0:
            v += 1 << w                       # two's complement
        assert 0 <= v < (1 << w), f"{name}.{s['name']}={raw_fields[s['name']]} does not fit {w} bits"
        for k, ch in enumerate(format(v, f"0{w}b")):
            bits[7 + s["bit"] + k] = ch
    return int("".join(bits), 2)


def decode(word):
    """Returns (dest, message_name, {signal: raw}) for a 64-bit word."""
    bits = format(word, "064b")
    dest = int(bits[0])
    mid = int(bits[1:7], 2)
    m = BY_ID.get(mid)
    if m is None:
        return dest, None, {}
    out = {}
    for s in m["signals"]:
        field = bits[7 + s["bit"]: 7 + s["bit"] + s["width"]]
        v = int(field, 2)
        if s["signed"] and field[0] == "1":
            v -= 1 << s["width"]
        out[s["name"]] = v
    return dest, m["name"], out


def to_bytes(word):
    return word.to_bytes(8, "big")


def from_bytes(b):
    return int.from_bytes(b, "big")
