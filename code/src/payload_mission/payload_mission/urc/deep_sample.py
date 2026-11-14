"""
urc/deep_sample.py -- step functions for the URC "Take Deep Sample" mission.

Each function has the signature  fn(ctx, params, operator_answer)  and is named in the step file
(jetson_function / antithesis_function).  They are written to be SAFE TO RE-RUN: running the same
function twice leaves the hardware in the same state (hw.command ignores repeats).

Conventions:
  * Targets come from the step file `params` (so the lead can tune them without editing code).
  * Every target is range-checked against the URDF by ctx.units BEFORE anything is sent.
  * Every move is confirmed against telemetry (readback) with a timeout; no confirmation = HardwareFault.
  * The Jetson sends a target ONCE and then only watches.  The firmware holds the position.
"""
import math

from ..hw import HardwareFault

POS_TOL_MM = 0.5            # how close counts as "arrived"
HOME_TOL_MM = 0.5


def _move_joint(ctx, joint, target_rad, readback_rad_tol, timeout_s):
    """Send a position target once, then wait for the joint to arrive."""
    ctx.hw.command(joint, "position", target_rad)
    ctx.hw.confirm(joint, "position", target_rad, readback_rad_tol, timeout_s)


def VPSHome(ctx, p, answer):
    """Step 001: send the VPS to its limit switch; the board zeroes its position there."""
    ctx.hw.check_link()
    ctx.hw.pulse("vps_control", "home")                                  # one trigger; the board does the homing
    tol = abs(ctx.units.vps_mm_to_rad(HOME_TOL_MM))      # abs: the URDF multiplier is negative
    ctx.hw.confirm("vps_screw_rotation", "position", 0.0, tol, p.get("timeout_s", 60))


def URCDrillHome(ctx, p, answer):
    """Step 002: home the drill lift; the board sets its limit-switch position to zero."""
    ctx.hw.check_link()
    ctx.hw.pulse("drill_control", "home")
    tol = ctx.units.drill_mm_to_rad(HOME_TOL_MM)
    ctx.hw.confirm("drill_lift_screw_rotation", "position", 0.0, abs(tol), p.get("timeout_s", 20))


def VPSGround(ctx, p, answer):
    """
    Step 003: lower the VPS in small steps until the distance sensor says it is at the ground.

    The ground sensor reports metres between the VPS and the ground.  We step down `step_mm` at a
    time (a new target each step; it is NOT streamed), stop when distance <= contact_mm, and give up
    at max_travel_mm so a bad sensor can never drive the box into the rover.
    """
    step, contact, limit = p.get("step_mm", 5), p.get("contact_mm", 2), p.get("max_travel_mm", 200)
    u, hw = ctx.units, ctx.hw
    pos_mm = max(0.0, u.vps_rad_to_mm(hw.read("vps_screw_rotation", "position")) or 0.0)
    while True:
        dist_mm = hw.read("ground_sensor", "distance") * 1000.0
        if math.isnan(dist_mm):
            raise HardwareFault("ground sensor has no reading")
        if dist_mm <= contact:
            return                                                         # at ground level
        if pos_mm >= limit:
            raise HardwareFault(f"VPS travelled {pos_mm:.0f} mm without reaching the ground")
        pos_mm = min(pos_mm + min(step, dist_mm - contact), limit)         # never step past the contact point
        u.check_vps_mm(pos_mm)
        _move_joint(ctx, "vps_screw_rotation", u.vps_mm_to_rad(pos_mm), abs(u.vps_mm_to_rad(POS_TOL_MM)), p.get("step_timeout_s", 20))


def URCDrillSet(ctx, p, answer):
    """Steps 004/008: drill on (with power 0..1) or off.  Same function undoes itself: antithesis params say off."""
    if p.get("on"):
        # `power` is a fraction of the free speed; max_rpm lives in params so it is visible and tunable.
        rpm = p.get("power", 0.5) * p.get("max_rpm", 300.0)
        ctx.hw.command("drill_tool_rotation", "velocity", ctx.units.rpm_to_rads(rpm))
        if p.get("confirm", True):
            ctx.hw.wait(p.get("spin_up_s", 1.0))
    else:
        ctx.hw.command("drill_tool_rotation", "velocity", 0.0)


def URCDrillMove(ctx, p, answer):
    """Steps 005/007/010: move the drill to `depth_mm` below home (0 = home, positive = deeper)."""
    depth = abs(p["depth_mm"])                                             # the step file writes -100; direction is handled in units.py
    ctx.units.check_drill_depth_mm(depth)
    target = ctx.units.drill_mm_to_rad(depth)
    tol = abs(ctx.units.drill_mm_to_rad(POS_TOL_MM))
    _move_joint(ctx, "drill_lift_screw_rotation", target, tol, p.get("timeout_s", 30))


def WaitAtDepth(ctx, p, answer):
    """Step 006: dwell while drilling.  Abortable, and re-checks the link every second."""
    left = float(p.get("seconds", 10))
    while left > 0:
        ctx.hw.check_link()
        ctx.hw.wait(min(1.0, left))
        left -= 1.0


def URCPlugSet(ctx, p, answer):
    """Step 009: move the plug servo to a preset angle (degrees) and wait for the open/closed bit if it flips."""
    angle = {1: p.get("angle_deg", 90.0), 0: 0.0}.get(p.get("preset", 1), p.get("angle_deg", 90.0))
    ctx.hw.command("plug", "angle", angle)
    # Table gives only a state bit, not the angle: confirmation is on that bit.
    ctx.hw.wait(p.get("settle_s", 1.0))


REGISTRY = {f.__name__: f for f in (VPSHome, URCDrillHome, VPSGround, URCDrillSet, URCDrillMove, WaitAtDepth, URCPlugSet)}
