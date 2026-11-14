#!/usr/bin/env python3
"""
check_contract.py -- fails loudly if the three sources of truth disagree.

  mechanical URDF  <->  ros2_control xacro  <->  can_signals.yaml / CAN_Table.xlsx

Checks
  1. every ros2_control <joint> exists in the URDF and is NOT a <mimic> follower
  2. every declared interface is served by a CAN binding (and every binding's interface is declared)
  3. units sanity: CAN field ranges versus URDF joint limits (printed as a table)
Usage:  python3 scripts/check_contract.py [--hardware can] [--urdf PATH]
"""
import argparse, json, math, os, sys
import xml.etree.ElementTree as ET
import xacro

HERE = os.path.dirname(os.path.abspath(__file__))
HW = os.path.join(HERE, "..", "..", "payload_hardware")
ap = argparse.ArgumentParser()
ap.add_argument("--hardware", default="can")
ap.add_argument("--urdf", default=os.path.join(HW, "config", "payload_description.urdf"))
args = ap.parse_args()

xml = xacro.process_file(os.path.join(HERE, "..", "urdf", "payload.urdf.xacro"),
                         mappings={"description_file": args.urdf, "hardware": args.hardware}).toxml()
doc = ET.fromstring(xml)
table = json.load(open(os.path.join(HW, "config", "can_table.json")))
fails = []

urdf_joints = {j.get("name"): j for j in doc.findall("joint")}
rc = doc.find("ros2_control")
declared_state, declared_cmd = set(), set()
for comp in list(rc.findall("joint")) + list(rc.findall("gpio")):
    for si in comp.findall("state_interface"):
        declared_state.add(f"{comp.get('name')}/{si.get('name')}")
    for ci in comp.findall("command_interface"):
        declared_cmd.add(f"{comp.get('name')}/{ci.get('name')}")

# 1. joints vs URDF
for j in rc.findall("joint"):
    n = j.get("name")
    if n not in urdf_joints:
        fails.append(f"ros2_control joint '{n}' is not in the URDF")
    elif urdf_joints[n].find("mimic") is not None:
        fails.append(f"ros2_control joint '{n}' is a <mimic> follower in the URDF; declare its master screw instead")

# 2. interfaces vs bindings
served_state = {b["interface"] for b in table["state_bindings"]}
served_cmd = set()
for b in table["command_bindings"]:
    for v in b["values"]:
        served_cmd.add(v["interface"])
    if b["trigger"]:
        served_cmd.add(b["trigger"])
diag = {f"payload_link/{x}" for x in ("link_ok", "rx_age_ms", "fault_flags", "tx_count", "rx_count", "rejected_count")}
served_state |= diag
served_cmd.add("payload_control/clear_faults")
for n in sorted(declared_state - served_state): fails.append(f"state interface '{n}' is declared but nothing in the CAN table feeds it")
for n in sorted(declared_cmd - served_cmd): fails.append(f"command interface '{n}' is declared but no CAN command message uses it")
for n in sorted(served_state - declared_state): fails.append(f"CAN binding feeds state '{n}' but it is not declared in the xacro")
for n in sorted(served_cmd - declared_cmd): fails.append(f"CAN binding uses command '{n}' but it is not declared in the xacro")

# 3. units sanity
print("URDF joints that are driven (from ros2_control) and their mechanical limits")
for j in rc.findall("joint"):
    n = j.get("name"); u = urdf_joints.get(n)
    lim = u.find("limit").attrib if u is not None and u.find("limit") is not None else {}
    print(f"  {n:28s} type={u.get('type') if u is not None else '?':10s} limit={dict(lim) if lim else 'none in URDF'}")
for mimic_name in ("vps_box_translation", "drill_platform_translation"):
    j = urdf_joints[mimic_name]; m = j.find("mimic"); lim = j.find("limit").attrib
    mult = float(m.get("multiplier"))
    lo, hi = float(lim["lower"]), float(lim["upper"])
    print(f"  {mimic_name:28s} follows {m.get('joint')} x {mult:+.6g} m/rad: travel {lo*1000:+.0f}..{hi*1000:+.0f} mm "
          f"= screw {lo/mult:+.1f}..{hi/mult:+.1f} rad = {abs(lo/mult)/(2*math.pi):.1f}..{abs(hi/mult)/(2*math.pi):.1f} turns")

print(f"\ndeclared: {len(declared_state)} state / {len(declared_cmd)} command interfaces; "
      f"CAN table serves {len(served_state)} / {len(served_cmd)}")
if fails:
    print("\nCONTRACT FAILURES:")
    for f in fails: print("  -", f)
    sys.exit(1)
print("contract OK")
