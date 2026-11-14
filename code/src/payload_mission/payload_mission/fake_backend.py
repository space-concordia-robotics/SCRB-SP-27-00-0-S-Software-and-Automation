"""
fake_backend.py -- a simulated payload for testing the mission layer with no ROS and no hardware.

It behaves like "ros2_control + hardware interface + firmware" as seen from hw.py: commands move
simulated devices at the speeds the URDF allows (scaled up by `speedup` so tests run in seconds),
and state interfaces report what the devices are doing.  Faults can be injected.
"""
import math

from .units import Units

NaN = float("nan")


class FakeBackend:
    def __init__(self, units: Units, speedup=100.0, ground_mm=50.0):
        self.u = units
        self.speedup = speedup
        self.ground_mm = ground_mm            # the ground is this far below the VPS home position
        self.cmd = {}                         # everything ever sent, for assertions
        self.sent = []
        self.stuck = set()                    # components that ignore commands (fault injection)
        self.s = {                            # state interfaces
            ("vps_screw_rotation", "position"): 0.0,
            ("drill_lift_screw_rotation", "position"): 0.0,
            ("carousel_rotation", "position"): 0.0,
            ("soil_sampler_rotation", "position"): 0.0,
            ("drill_tool_rotation", "velocity"): 0.0,
            ("plug", "state"): 0.0,
            ("payload_link", "link_ok"): 1.0,
            ("payload_link", "fault_flags"): 0.0,
        }
        self.target = {}                      # joint -> target position
        self.homing = {}                      # joint -> True while a home is in progress
        self._counter = {}

    # ---- what hw.py calls ----
    def send(self, comp, iface, value):
        self.sent.append((comp, iface, value))
        if comp in self.stuck:
            return
        key = (comp, iface)
        if key[1] == "position" and comp in ("vps_screw_rotation", "drill_lift_screw_rotation", "carousel_rotation", "soil_sampler_rotation"):
            self.target[comp] = value
        elif key == ("drill_tool_rotation", "velocity"):
            self.s[key] = value
        elif iface in ("power", "on"):
            self.s[(comp, "running" if comp.startswith("pump") else "on")] = 1.0 if value != 0 else 0.0
        elif key == ("plug", "angle"):
            self.s[("plug", "state")] = 1.0 if value > 0 else 0.0
        elif iface == "home":                 # counter changed -> go home
            self.target["vps_screw_rotation" if comp == "vps_control" else "drill_lift_screw_rotation"] = 0.0
        self.cmd[key] = value

    def get(self, comp, iface):
        if (comp, iface) == ("ground_sensor", "distance"):
            mm = self.u.vps_rad_to_mm(self.s[("vps_screw_rotation", "position")])
            return max(0.0, (self.ground_mm - mm)) / 1000.0
        return self.s.get((comp, iface), NaN)

    def poll(self, dt):
        """Advance simulated time by dt (x speedup)."""
        sim_dt = dt * self.speedup
        speeds = {   # rad/s from the URDF velocity limits
            "vps_screw_rotation": 0.00615 / abs(self.u.vps_mult),
            "drill_lift_screw_rotation": 0.004667 / abs(self.u.drill_mult),
            "carousel_rotation": 1.0, "soil_sampler_rotation": 1.0}
        for j, tgt in self.target.items():
            cur = self.s[(j, "position")]
            step = speeds[j] * sim_dt
            self.s[(j, "position")] = tgt if abs(tgt - cur) <= step else cur + math.copysign(step, tgt - cur)
        import time
        time.sleep(dt)
