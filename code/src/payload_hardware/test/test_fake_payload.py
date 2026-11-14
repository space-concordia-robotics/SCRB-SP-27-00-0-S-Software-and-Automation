"""Logic test for the fake board (no socket needed): commands move it, telemetry reports it."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))
import tools.can_table as ct
from tools.fake_payload import FakePayload

fp = FakePayload()
fp.handle(ct.encode("VPS_MOVE", 0, {"target_displacement": 60}))
for _ in range(100): fp.step(0.1)                     # 10 s at 6.15 mm/s -> reached 60 mm
t = fp.telemetry()
_, name, f = ct.decode(t["TELEM_URC_GENERAL"])
assert name == "TELEM_URC_GENERAL" and f["vps_displacement"] == 60, f
fp.handle(ct.encode("URC_PUMP_RUN", 0, {"selector": 1, "power": -5000}))
fp.handle(ct.encode("LED_SET", 0, {"selector": 1, "on": 1}))
_, _, a = ct.decode(fp.telemetry()["TELEM_URC_ANALYZE"])
assert a["pump2_status"] == 1 and a["pump1_status"] == 0 and a["led_445"] == 1 and a["led_405"] == 0, a
fp.handle(ct.encode("URC_DSS_MOVE", 0, {"target_displacement": 100}))
for _ in range(300): fp.step(0.1)
_, _, d = ct.decode(fp.telemetry()["TELEM_URC_DEEP_SAMPLE"])
assert d["drill_disp"] == 100, d
fp.handle(ct.encode("KILL", 0, {"kill": 1}))
fp.handle(ct.encode("VPS_MOVE", 0, {"target_displacement": 0}))
for _ in range(100): fp.step(0.1)
_, _, g = ct.decode(fp.telemetry()["TELEM_URC_GENERAL"])
assert g["vps_displacement"] == 60, "after KILL the fake must not move"
print("fake payload logic OK")
