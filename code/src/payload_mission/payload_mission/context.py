"""
context.py -- what every step function receives: hardware (hw), unit conversion (units) and settings.
Keeping these together means a step function has one argument to learn, and tests can swap them out.
"""
from dataclasses import dataclass, field

from .hw import HW
from .units import Units

# What to command when stopping.  (component, interface, stop value).  Only outputs we have actually driven are touched.
URC_STOPPERS = [
    ("drill_tool_rotation", "velocity", 0.0),
    ("pump_1", "power", 0.0), ("pump_2", "power", 0.0),
    ("ultrasonic_mixer", "on", 0.0), ("led_405", "on", 0.0), ("led_445", "on", 0.0),
]
# Position-controlled axes: "stop" = retarget to where they are now.
URC_MOVING_JOINTS = [("vps_screw_rotation", "position"), ("drill_lift_screw_rotation", "position")]


@dataclass
class Ctx:
    hw: HW
    units: Units
    competition: str = "urc"
    stoppers: list = field(default_factory=lambda: list(URC_STOPPERS))
    moving: list = field(default_factory=lambda: list(URC_MOVING_JOINTS))

    def safe_state(self):
        """Stop outputs and freeze axes.  Called on abort/fault and when a run ends abnormally."""
        self.hw.halt_motion(self.moving)
        self.hw.safe_state(self.stoppers)
