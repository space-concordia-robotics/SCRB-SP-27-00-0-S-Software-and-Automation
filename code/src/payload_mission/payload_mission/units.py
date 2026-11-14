"""
units.py -- turns mission units (mm, rpm, degrees) into ros2_control units (rad, rad/s) and back.

WHERE THE NUMBERS COME FROM
  The mechanical URDF has two <mimic> joints:  a screw turns, a box/platform translates.
      translation_m = multiplier x screw_rotation_rad
  Reading the multiplier from the URDF (never typing it) means a new CAD export updates the
  code automatically.  payload_hardware/config/can_signals.yaml reads the SAME multiplier, so
  the mission layer and the hardware interface can never disagree about what 1 rad means
  (tests/test_units.py proves they agree).

SIGN CONVENTIONS (ASSUMPTION A4 in payload_hardware/docs/CAN_TABLE_REPORT.md; confirm with mechanical)
  VPS   : "displacement" in mm is +translation of vps_box_translation, 0 = home, larger = lower.
  Drill : "depth" in mm is -translation of drill_platform_translation, 0 = home, larger = deeper.
"""
import math
import xml.etree.ElementTree as ET


class Units:
    def __init__(self, urdf_path):
        joints = {}
        for j in ET.parse(urdf_path).getroot().findall("joint"):
            m, lim = j.find("mimic"), j.find("limit")
            joints[j.get("name")] = dict(
                mult=float(m.get("multiplier")) if m is not None else None,
                lower=float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
                upper=float(lim.get("upper")) if lim is not None and lim.get("upper") else None)
        self.vps_mult = joints["vps_box_translation"]["mult"]            # m of box travel per rad of screw
        self.drill_mult = joints["drill_platform_translation"]["mult"]   # m of platform travel per rad of screw
        vps, drill = joints["vps_box_translation"], joints["drill_platform_translation"]
        # Mechanical travel from the URDF (this is tighter than what the CAN field could carry).
        self.vps_range_mm = (max(0.0, vps["lower"] * 1000), vps["upper"] * 1000)          # 0 .. 305
        self.drill_depth_range_mm = (0.0, -drill["lower"] * 1000)                           # 0 .. 160

    def vps_mm_to_rad(self, mm):   return (mm / 1000.0) / self.vps_mult
    def vps_rad_to_mm(self, rad):  return rad * self.vps_mult * 1000.0

    def drill_mm_to_rad(self, depth_mm):  return -(depth_mm / 1000.0) / self.drill_mult
    def drill_rad_to_mm(self, rad):       return -rad * self.drill_mult * 1000.0

    @staticmethod
    def rpm_to_rads(rpm):   return rpm * 2 * math.pi / 60.0
    @staticmethod
    def rads_to_rpm(rads):  return rads * 60.0 / (2 * math.pi)
    @staticmethod
    def deg_to_rad(deg):    return math.radians(deg)
    @staticmethod
    def rad_to_deg(rad):    return math.degrees(rad)
    @staticmethod
    def cuvette_to_rad(i):  return i * 2 * math.pi / 12.0
    @staticmethod
    def rad_to_cuvette(r):  return r * 12.0 / (2 * math.pi)

    def check_vps_mm(self, mm):
        lo, hi = self.vps_range_mm
        if not (lo <= mm <= hi):
            raise ValueError(f"VPS target {mm} mm outside the URDF travel {lo:.0f}..{hi:.0f} mm")
        return mm

    def check_drill_depth_mm(self, mm):
        lo, hi = self.drill_depth_range_mm
        if not (lo <= mm <= hi):
            raise ValueError(f"drill depth {mm} mm outside the URDF travel {lo:.0f}..{hi:.0f} mm")
        return mm
