"""Run:  python3 tests/test_urc.py   (no ROS needed)"""
import os, sys, threading, time
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from payload_mission.units import Units
from payload_mission.fake_backend import FakeBackend
from payload_mission.mission_node_urc import UrcMission
from payload_mission.hw import HW, Aborted, HardwareFault
from payload_mission.hw import fill_group_command
import payload_mission.urc.deep_sample as ds

URDF = f"{ROOT}/config/payload_description.urdf"
u = Units(URDF)
n = 0
def check(c, m):
    global n; n += 1
    assert c, m

# ---- units ----
check(abs(u.vps_mult + 0.0006992351524728963) < 1e-15, "vps mult")
check(abs(u.vps_rad_to_mm(u.vps_mm_to_rad(37.0)) - 37.0) < 1e-9, "vps round trip")
check(abs(u.drill_rad_to_mm(u.drill_mm_to_rad(80.0)) - 80.0) < 1e-9, "drill round trip")
check(u.drill_mm_to_rad(50) > 0, "drill depth is positive screw rotation")  # multiplier negative -> depth positive
check(u.vps_range_mm == (0.0, 305.0) and u.drill_depth_range_mm == (0.0, 160.0), "ranges")
for bad in (lambda: u.check_vps_mm(400), lambda: u.check_drill_depth_mm(170)):
    try: bad(); check(False, "should reject")
    except ValueError: check(True, "")
check(abs(u.rpm_to_rads(60) - 6.283185307) < 1e-6, "rpm")

# ---- agreement with the hardware package's scale expressions (same URDF number) ----
import yaml
y = open(f"{ROOT}/../payload_hardware/config/can_signals.yaml").read()
check("mimic('vps_box_translation')" in y and "mimic('drill_platform_translation')" in y, "hw yaml uses URDF mimic")

# ---- group command vector ----
J = ["a", "b", "c"]
check(fill_group_command(J, {"b": 1.0}, {"a": 5.0}, {"c": 0.3}) == [5.0, 1.0, 0.3], "fill keeps previous cmd, uses state for new")
nan = float("nan")
check(fill_group_command(J, {}, {}, {"a": nan}) == [0.0, 0.0, 0.0], "nan state -> 0")

def mission(**kw):
    b = FakeBackend(u, **kw); m = UrcMission(b, URDF, f"{ROOT}/missions"); return b, m

# ---- send on change ----
b = FakeBackend(u); hw = HW(b)
hw.command("pump_1", "power", 0.5); hw.command("pump_1", "power", 0.5); hw.command("pump_1", "power", 0.6)
check(len(b.sent) == 2, "send-on-change")
hw.pulse("vps_control", "home"); hw.pulse("vps_control", "home")
check([s[2] for s in b.sent if s[1] == "home"] == [1.0, 2.0], "pulse counter increments")

# ---- full happy path ----
b, m = mission(speedup=400)
seen = []
m.progress = lambda sid, st: seen.append((sid, st))
t = time.time(); res = m.run("urc_take_deep_sample")
check(res == "COMPLETE", f"happy path -> {res} {getattr(m.runner,'error','')}")
check(seen[-1] == ("URC-DS-010", "COMPLETE"), "last step")
check(abs(u.vps_rad_to_mm(b.s[("vps_screw_rotation", "position")]) - 48.0) < 1.0, "VPS ended ~48mm (ground 50, contact 2)")
check(b.s[("drill_tool_rotation", "velocity")] == 0.0, "drill off at end")
vel = [s[2] for s in b.sent if s[0] == "drill_tool_rotation"]
check(len(vel) == 2 and vel[0] > 0, "drill commanded on once, off once")
check(max(s[2] for s in b.sent if s[0] == "drill_lift_screw_rotation") <= u.drill_mm_to_rad(160), "never past URDF travel")

# ---- fault: drill lift ignores commands -> FAULT, drill stopped, VPS rolled back ----
b, m = mission(speedup=400); b.stuck.add("drill_lift_screw_rotation")
m.progress = lambda sid, st: seen.append((sid, st)); seen.clear()
orig = ds.URCDrillMove
res = m.run("urc_take_deep_sample")
check(res == "FAULT", f"stuck -> {res}")
check(b.s[("drill_tool_rotation", "velocity")] == 0.0, "drill stopped after fault")
check(("URC-DS-003", "REVERSED") in seen, "VPS ground step reversed")
check(("URC-DS-004", "REVERSED") in seen, "drill-on reversed (turned off)")
check(("URC-DS-002", "REVERSED") not in seen, "irreversible step not reversed")

# ---- abort during dwell ----
b, m = mission(speedup=400)
def killer():
    while not any(s[0] == "drill_tool_rotation" and s[2] > 0 for s in b.sent): time.sleep(0.005)
    time.sleep(0.05); m.request_abort()
threading.Thread(target=killer, daemon=True).start()
res = m.run("urc_take_deep_sample")
check(res in ("ABORTED",), f"abort -> {res}")
check(b.s[("drill_tool_rotation", "velocity")] == 0.0, "drill off after abort")

# ---- link down -> fault ----
b, m = mission(speedup=400); b.s[("payload_link", "link_ok")] = 0.0
check(m.run("urc_take_deep_sample") == "FAULT", "link down -> FAULT before anything moves")
check(not any(s[2] != 0 for s in b.sent if s[0] == "drill_tool_rotation"), "nothing driven")

# ---- ground sensor dead (NaN) ----
b, m = mission(speedup=400); b.get_orig = b.get
b.get = lambda c, i: float("nan") if c == "ground_sensor" else b.get_orig(c, i)
check(m.run("urc_take_deep_sample") == "FAULT", "NaN sensor -> FAULT")

# ---- sensor never reaches ground (ground very far) -> gives up at max travel ----
b, m = mission(speedup=400, ground_mm=1000)
r = m.run("urc_take_deep_sample")
check(r == "FAULT", f"no ground -> {r}")
check(u.vps_rad_to_mm(max(s[2] for s in b.sent if s[0] == "vps_screw_rotation" and s[1] == "position")) <= 200.01, "VPS capped at max_travel")

# ---- unknown mission ----
try: m.run("nope"); check(False, "")
except KeyError: check(True, "")

print(f"mission layer OK ({n} checks)")
