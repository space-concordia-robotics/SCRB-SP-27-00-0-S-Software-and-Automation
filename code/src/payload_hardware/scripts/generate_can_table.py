#!/usr/bin/env python3
"""
generate_can_table.py -- CAN_Table.xlsx + can_signals.yaml + URDF  ->  C++ header, JSON, report.

Why this exists
---------------
The CAN table (Excel) is the contract with the firmware team. Hand-typing it into C++
would let the two drift apart silently.  This script:
  1. reads every message row from the spreadsheet (id, destination, size, bit mask, rate),
  2. loads the reviewed field map in can_signals.yaml,
  3. CROSS-CHECKS them (ids, names, destinations, bit sizes, masks, overlaps),
  4. evaluates the scale expressions using constants read from the URDF (mimic multipliers),
  5. writes include/payload_hardware/can_table_generated.hpp  (used by the C++ core),
     config/can_table.json                                      (used by Python tools),
     docs/CAN_TABLE_REPORT.md                                   (every mismatch / assumption).
Run:  python3 scripts/generate_can_table.py   (from the package root)
"""
import json, math, os, re, sys
import openpyxl, yaml
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XLSX = os.path.join(ROOT, "config", "CAN_Table.xlsx")
YAML = os.path.join(ROOT, "config", "can_signals.yaml")
URDF = os.path.join(ROOT, "config", "payload_description.urdf")
OUT_HPP = os.path.join(ROOT, "include", "payload_hardware", "can_table_generated.hpp")
OUT_JSON = os.path.join(ROOT, "config", "can_table.json")
OUT_REPORT = os.path.join(ROOT, "docs", "CAN_TABLE_REPORT.md")

problems, notes = [], []          # problems = must fix; notes = assumptions/ambiguities to confirm


# ---------------------------------------------------------------- spreadsheet
def read_sheet():
    ws = openpyxl.load_workbook(XLSX, data_only=True)["Can Table"]
    rows = {}
    for r in range(2, ws.max_row + 1):
        cell = ws.cell(r, 1).value
        if not isinstance(cell, str) or " " not in cell:
            continue
        head, payload = cell.split(" ", 1)
        msg_id = int(head[-6:], 2) if re.fullmatch(r"[01]{6}", head[-6:]) else None
        if msg_id is None:
            continue
        mask = payload.strip()
        used = [i for i, ch in enumerate(mask) if ch == "1"]
        rows[msg_id] = dict(
            row=r, head=head, head_len=len(head), mask=mask,
            dest=ws.cell(r, 2).value, desc=ws.cell(r, 3).value,
            bits=ws.cell(r, 4).value, text=ws.cell(r, 5).value,
            freq=ws.cell(r, 6).value, name=ws.cell(r, 7).value,
            used_bits=(max(used) + 1 if used else 0), n_ones=len(used),
            dest_char=(head[28] if len(head) == 35 else None))
    return rows


# ---------------------------------------------------------------- URDF constants
def read_urdf():
    tree = ET.parse(URDF)
    joints = {}
    for j in tree.getroot().findall("joint"):
        info = dict(type=j.get("type"), mimic=None, limit={})
        m = j.find("mimic")
        if m is not None:
            info["mimic"] = dict(joint=m.get("joint"), multiplier=float(m.get("multiplier", 1)),
                                 offset=float(m.get("offset", 0)))
        lim = j.find("limit")
        if lim is not None:
            info["limit"] = {k: float(v) for k, v in lim.attrib.items()}
        joints[j.get("name")] = info
    return joints


def make_eval(joints):
    def mimic(name):
        if name not in joints or not joints[name]["mimic"]:
            raise KeyError(f"URDF joint '{name}' has no <mimic>")
        return joints[name]["mimic"]["multiplier"]
    def limit(name, which):
        return joints[name]["limit"][which]
    env = {"__builtins__": {}, "pi": math.pi, "abs": abs, "mimic": mimic, "limit": limit}
    def ev(x, default):
        if x is None:
            return default
        if isinstance(x, (int, float)):
            return float(x)
        return float(eval(str(x), env))          # trusted project config, not user input
    return ev


# ---------------------------------------------------------------- main
def main():
    sheet = read_sheet()
    cfg = yaml.safe_load(open(YAML))
    joints = read_urdf()
    ev = make_eval(joints)
    msgs = cfg["messages"]
    by_name = {m["name"]: (i, m) for i, m in msgs.items()}

    # ---- frame layout
    w = cfg["frame"]["id_widths"]; f = cfg["frame"]["id_fields"]
    shift, arb = 0, 0
    for fld in reversed(["device_type", "api_class", "reserved_one", "api_index", "device_id"]):
        arb |= (f[fld] << shift); shift += w[fld]
    if shift != 29:
        problems.append(f"id field widths sum to {shift}, expected 29 bits")

    # ---- cross-check messages against the sheet
    for mid, m in sorted(msgs.items()):
        s = sheet.get(mid)
        if not s:
            problems.append(f"message {mid} {m['name']}: not found in the spreadsheet"); continue
        want_dest = "JETSON" if m["dir"] == "telemetry" else "PAYLOAD"
        if s["dest"] and s["dest"] != want_dest:
            problems.append(f"{m['name']}: sheet destination {s['dest']} vs yaml dir {m['dir']}")
        if s["name"] and s["name"] != "Telemetry" and s["name"].lower() != m["name"].lower():
            problems.append(f"id {mid}: sheet name {s['name']} vs yaml {m['name']}")
        if s["head_len"] != 35:
            notes.append(f"id {mid} {m['name']}: header string is {s['head_len']} chars (normal rows are 35); the sheet marks it 'ERROR' -- layout undefined")
        # signal extents vs mask
        sig_bits = set()
        for sg in m["signals"]:
            rng = set(range(sg["bit"], sg["bit"] + sg["width"]))
            if rng & sig_bits:
                problems.append(f"{m['name']}.{sg['name']}: overlaps another signal")
            sig_bits |= rng
            if sg["bit"] + sg["width"] > 57:
                problems.append(f"{m['name']}.{sg['name']}: exceeds the 57-bit payload")
        mask_bits = {i for i, ch in enumerate(s["mask"]) if ch == "1"}
        if m["signals"] and sig_bits != mask_bits:
            problems.append(f"{m['name']}: yaml signal bits {sorted(sig_bits)[:3]}.. ({len(sig_bits)} bits) differ from sheet mask ({len(mask_bits)} bits)")
        if s["bits"] is not None and m["signals"] and int(s["bits"]) != len(sig_bits):
            notes.append(f"{m['name']}: sheet 'payload bit size' column says {s['bits']} but mask/yaml use {len(sig_bits)}")
    for mid in sheet:
        if mid not in msgs:
            problems.append(f"spreadsheet message id {mid} is missing from can_signals.yaml")

    # ---- signals: raw ranges
    def sig_range(sg):
        if sg.get("signed"):
            lo, hi = -(1 << (sg["width"] - 1)), (1 << (sg["width"] - 1)) - 1
        else:
            lo, hi = 0, (1 << sg["width"]) - 1
        return max(lo, sg.get("min", lo)), min(hi, sg.get("max", hi))

    def sig_index(mname, sname):
        _, m = by_name[mname]
        for i, sg in enumerate(m["signals"]):
            if sg["name"] == sname:
                return i
        raise KeyError(f"{mname}.{sname}")

    # ---- state bindings
    state_b = []
    for b in cfg["state_bindings"]:
        mname, sname = b["signal"].split(".")
        mid = by_name[mname][0]
        state_b.append(dict(interface=b["interface"], message_id=mid, message=mname, signal=sname,
                            signal_index=sig_index(mname, sname),
                            scale=ev(b.get("scale"), 1.0), offset=ev(b.get("offset"), 0.0)))

    # ---- command bindings
    cmd_b = []
    for b in cfg["command_bindings"]:
        mid, m = by_name[b["message"]]
        if m["dir"] != "command":
            problems.append(f"command binding uses telemetry message {b['message']}")
        consts = [dict(signal_index=sig_index(b["message"], k), signal=k, raw=int(v))
                  for k, v in (b.get("constants") or {}).items()]
        values = [dict(signal_index=sig_index(b["message"], k), signal=k, interface=v["interface"],
                       scale=ev(v.get("scale"), 1.0), offset=ev(v.get("offset"), 0.0))
                  for k, v in (b.get("values") or {}).items()]
        covered = {c["signal"] for c in consts} | {v["signal"] for v in values}
        missing = [sg["name"] for sg in m["signals"] if sg["name"] not in covered]
        if missing:
            problems.append(f"{b['message']} binding does not set signals {missing}")
        rb = b.get("readback")
        cmd_b.append(dict(message_id=mid, message=b["message"], consts=consts, values=values,
                          trigger=b.get("trigger", ""),
                          readback_state=(rb["state"] if rb else ""),
                          tolerance=(ev(rb.get("tolerance"), 0.0) if rb else 0.0),
                          timeout_ms=(int(rb.get("timeout_ms", 5000)) if rb else 0),
                          skip_initial=bool(b.get("skip_initial_if_at_readback", False))))

    # ---- safe-state commands
    safe = []
    for s in cfg["safe_state_commands"]:
        mid, m = by_name[s["message"]]
        safe.append(dict(message_id=mid, message=s["message"],
                         raws=[dict(signal_index=sig_index(s["message"], k), raw=int(v)) for k, v in s["raw"].items()]))

    # ---- sanity: command ranges vs URDF limits (informational)
    vps_max = limit_mm = joints["vps_box_translation"]["limit"]
    notes.append(f"VPS_MOVE accepts 0-1000 mm but the URDF translation limit is {vps_max['lower']*1000:.0f}..{vps_max['upper']*1000:.0f} mm: "
                 "the mission layer must clamp to the URDF limits (the CAN field alone would allow 1000 mm).")
    dl = joints["drill_platform_translation"]["limit"]
    notes.append(f"URC_DSS_MOVE accepts 0-200 mm but the URDF drill travel is {dl['lower']*1000:.0f}..{dl['upper']*1000:.0f} mm "
                 f"(only {-dl['lower']*1000:.0f} mm of depth).")

    # ---- static assumptions to surface
    notes.extend(STATIC_NOTES)

    # ---------------- emit JSON
    out = dict(
        frame=dict(arbitration_id=arb, dest_bits=1, msg_id_bits=6, payload_bits=57,
                   dest_jetson=cfg["frame"]["destination"]["jetson"], dest_payload=cfg["frame"]["destination"]["payload"]),
        messages={str(i): dict(name=m["name"], dir=m["dir"], competitions=m["competitions"],
                               rate_hz=sheet[i]["freq"] if isinstance(sheet[i]["freq"], (int, float)) else None,
                               signals=[dict(name=s["name"], bit=s["bit"], width=s["width"],
                                             signed=bool(s.get("signed")), raw_min=sig_range(s)[0], raw_max=sig_range(s)[1])
                                        for s in m["signals"]])
                  for i, m in msgs.items()},
        state_bindings=state_b, command_bindings=cmd_b, safe_state_commands=safe)
    json.dump(out, open(OUT_JSON, "w"), indent=1)

    # ---------------- emit C++
    def cstr(x): return '"' + x.replace('"', '\\"') + '"'
    def d(x): return repr(float(x))
    L = []
    L.append("// GENERATED by scripts/generate_can_table.py -- DO NOT EDIT.")
    L.append("// Sources: config/CAN_Table.xlsx, config/can_signals.yaml, config/payload_description.urdf")
    L.append("#pragma once\n#include <vector>\n#include \"payload_hardware/can_types.hpp\"\n")
    L.append("namespace payload_hardware { namespace can {\n")
    L.append(f"constexpr uint32_t kArbitrationId = 0x{arb:X}u;   // every frame uses this 29-bit id; the message id lives in the data\n")
    for mid, m in sorted(msgs.items()):
        L.append(f"constexpr uint8_t kMsg_{m['name']} = {mid};")
    L.append("")
    for mid, m in sorted(msgs.items()):
        if m["signals"]:
            L.append(f"inline constexpr SignalDef kSig_{m['name']}[] = {{")
            for sg in m["signals"]:
                lo, hi = sig_range(sg)
                L.append(f"  {{{cstr(sg['name'])}, {sg['bit']}, {sg['width']}, {str(bool(sg.get('signed'))).lower()}, {lo}LL, {hi}LL, {str(bool(sg.get('na_all_ones'))).lower()}}},")
            L.append("};")
    L.append("\ninline constexpr MessageDef kMessages[] = {")
    for mid, m in sorted(msgs.items()):
        comp = " | ".join({"urc": "kCompUrc", "erc": "kCompErc"}[c] for c in m["competitions"])
        sigs = f"kSig_{m['name']}" if m["signals"] else "nullptr"
        L.append(f"  {{{mid}, {cstr(m['name'])}, {str(m['dir']=='command').lower()}, {comp}, {sigs}, {len(m['signals'])}}},")
    L.append("};")
    L.append(f"inline constexpr int kNumMessages = {len(msgs)};\n")
    L.append("inline std::vector<StateBindingDef> state_bindings() {\n  return {")
    for b in state_b:
        L.append(f"    {{{cstr(b['interface'])}, {b['message_id']}, {b['signal_index']}, {d(b['scale'])}, {d(b['offset'])}}},")
    L.append("  };\n}\n")
    L.append("inline std::vector<CommandBindingDef> command_bindings() {\n  return {")
    for b in cmd_b:
        consts = ", ".join(f"{{{c['signal_index']}, {c['raw']}LL}}" for c in b["consts"])
        vals = ", ".join(f"{{{v['signal_index']}, {cstr(v['interface'])}, {d(v['scale'])}, {d(v['offset'])}}}" for v in b["values"])
        L.append(f"    {{{b['message_id']}, {{{consts}}}, {{{vals}}}, {cstr(b['trigger'])}, {cstr(b['readback_state'])}, {d(b['tolerance'])}, {b['timeout_ms']}, {str(b['skip_initial']).lower()}}},  // {b['message']}")
    L.append("  };\n}\n")
    L.append("inline std::vector<SafeCommandDef> safe_state_commands() {\n  return {")
    for s in safe:
        raws = ", ".join(f"{{{r['signal_index']}, {r['raw']}LL}}" for r in s["raws"])
        L.append(f"    {{{s['message_id']}, {{{raws}}}}},  // {s['message']}")
    L.append("  };\n}\n")
    L.append("}}  // namespace payload_hardware::can")
    open(OUT_HPP, "w").write("\n".join(L) + "\n")

    # ---------------- report
    R = ["# CAN table report (generated)\n",
         "Generated by `scripts/generate_can_table.py`. Re-run after any change to `CAN_Table.xlsx` or `can_signals.yaml`.\n",
         f"- Messages in sheet: {len(sheet)}; in yaml: {len(msgs)}",
         f"- Arbitration id used for every frame: `0x{arb:X}` (29-bit extended)",
         f"- State bindings: {len(state_b)}, command bindings: {len(cmd_b)}, safe-state commands: {len(safe)}\n",
         "## Problems (generation fails if any)\n"] + ([f"- {p}" for p in problems] or ["- none"])
    R += ["\n## Assumptions and ambiguities to confirm with electrical / firmware\n"] + [f"{i+1}. {n}" for i, n in enumerate(notes)]
    R += ["\n## Message list\n", "| id | name | dir | competitions | rate | payload bits |", "|---|---|---|---|---|---|"]
    for mid, m in sorted(msgs.items()):
        R.append(f"| {mid} | {m['name']} | {m['dir']} | {','.join(m['competitions'])} | {sheet[mid]['freq']} | {sum(s['width'] for s in m['signals'])} |")
    open(OUT_REPORT, "w").write("\n".join(R) + "\n")

    print(f"messages {len(msgs)}, state bindings {len(state_b)}, command bindings {len(cmd_b)}")
    print(f"problems: {len(problems)}, notes: {len(notes)}")
    for p in problems:
        print("PROBLEM:", p)
    return 1 if problems else 0


STATIC_NOTES = [
    "A1 (frame split): the sheet's numeric row gives header fields 5+6+1+7+10 = 29 bits (the CAN extended id) and a 64-bit data word = destination(1) + message id(6) + payload(57). The binary strings in the sheet are 35 characters, one short of 36, and the template shows the API-class field as 5 zeros although its width is 6. We assume the strings dropped one zero from the class field. Either way the only set id bit is the 'reserved one' bit, so the arbitration id is 0x20000.",
    "A2 (destination): we assume the destination bit is the first data bit (as the sheet groups it under 'Message Payload'), NOT part of the arbitration id. If firmware instead puts it in the id, only `kArbitrationId` and one filter change.",
    "A3 (bit order inside a field): the sheet says 'Bits 0-9' but does not say whether bit 0 is the most or least significant bit of the number. We assume bit 0 is the MOST significant bit (reading the mask strings left to right). This one needs a firmware answer before real hardware is used.",
    "A4 (displacement datum): the sheet gives VPS/drill 'displacement' in mm. We assume VPS displacement = URDF prismatic translation of vps_box_translation and drill depth = minus the translation of drill_platform_translation (down is negative). Mechanical must confirm the zero point and direction.",
    "No command message exists for the two solenoids or the lid servo (telemetry only), so they are read-only in the hardware interface.",
    "URC_PUMP_RUN has no duration field, so pump runs are timed by the Jetson (start, wait, stop). Only SPOOL_ROTATE carries a duration (0.1 s units).",
    "Carousel: command field is 'cuvette 0-11' (12 positions) but telemetry says 'Carousel position 0-12' (13 values) in a 4-bit field. We treat 12 positions, index = raw, angle = raw * 2*pi/12.",
    "Current mission: SET_MISSION is 3 bits (0-7) but telemetry says 'stated as 0-8'. Treated as 0-7.",
    "Telemetry 'Drill vertical displacement' is 7 bits (max 127 mm, sheet says 0-120) while URC_DSS_MOVE accepts 0-200 mm: depths above 127 mm cannot be reported back.",
    "URC_DSS_DRILL_SET accepts 0-1200 rpm but telemetry drill rotation is 7 bits (0-100 rpm): readback can never confirm speeds above 127 rpm.",
    "'dg' units on rock/soil mass are treated as decigrams (0.1 g). Temperature raw value is treated as degrees C directly (the sheet says range 10-600 in 10 bits).",
    "The sheet says LED wavelengths 405 nm and 445 nm; earlier project documents said 450 nm.",
    "Message ids 38 and 39 are labelled ERROR with an undefined header layout. They are ignored.",
    "CONFIGURATION (id 37, 1 Hz, destination Jetson, no payload) is used as a liveness signal together with the 20 Hz telemetry. There is no Jetson-to-payload heartbeat in the table, so a firmware watchdog cannot be satisfied yet. KILL exists and is exposed as payload_control/kill.",
    "Telemetry for the other competition: the lead said unused telemetry will read as NaN/NA. The encoding is unknown, so the hardware interface forces NaN for messages outside the configured competition and has an optional per-signal 'na_all_ones' switch for a sentinel.",
]

if __name__ == "__main__":
    sys.exit(main())
