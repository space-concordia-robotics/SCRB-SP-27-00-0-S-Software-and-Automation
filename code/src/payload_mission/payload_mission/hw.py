"""
hw.py -- the ONLY place the mission layer touches ros2_control.

Mission code never publishes topics or reads joint states itself.  It calls:

    hw.command("pump_1", "power", 0.8)       # send once; repeated identical calls do nothing
    hw.read("photodiode", "counts")
    hw.confirm("pump_1", "running", 1.0, tol=0.5, timeout_s=2)
    hw.pulse("vps_control", "home")          # trigger-style commands (counter +1)

Two ideas run through this file (both from the lead's review):
  1. The Jetson never "holds" anything.  A command is sent when the target CHANGES; the boards hold.
  2. Every command can be CONFIRMED against the telemetry that comes back.  A device that does not
     do what it was told is a fault, not a silent success.

Backends: RosBackend talks to the controllers; FakeBackend (in fake_backend.py) is a simulator for tests.
"""
import math
import threading
import time


class Aborted(Exception):
    """The operator pressed abort."""


class HardwareFault(Exception):
    """A device did not do what was commanded, or reported something impossible."""


class HW:
    def __init__(self, backend, abort_event=None, poll_s=0.02):
        self.backend = backend
        self.abort = abort_event or threading.Event()
        self.poll_s = poll_s
        self._last = {}           # (component, interface) -> last value we sent: the "send on change" memory
        self._driven = []         # outputs we have driven, so safe_state() knows what to stop
        self._counters = {}       # trigger-style commands

    #commands
    def command(self, comp, iface, value):
        """Send `value` unless it is exactly what we sent last time."""
        key = (comp, iface)
        if self._last.get(key) == value:
            return
        self._last[key] = value
        self.backend.send(comp, iface, float(value))
        if key not in self._driven:
            self._driven.append(key)

    def pulse(self, comp, iface):
        """Trigger-style command (home, zero, kill): the hardware interface fires once per counter change."""
        key = (comp, iface)
        self._counters[key] = self._counters.get(key, 0.0) + 1.0
        self.backend.send(comp, iface, self._counters[key])

    #readback
    def read(self, comp, iface):
        return self.backend.get(comp, iface)

    def confirm(self, comp, iface, target, tol, timeout_s):
        """Wait until the measured value is within `tol` of `target`; otherwise raise HardwareFault.
        NaN (no telemetry / not applicable) never counts as confirmation."""
        t0 = time.monotonic()
        while True:
            self._check_abort()
            v = self.backend.get(comp, iface)
            if not math.isnan(v) and abs(v - target) <= tol:
                return v
            if time.monotonic() - t0 > timeout_s:
                raise HardwareFault(f"{comp}/{iface}: wanted {target:.4g} (+/-{tol:.3g}), measured {v:.4g} after {timeout_s}s")
            self.backend.poll(self.poll_s)

    def check_link(self):
        "Raise if the hardware interface reports a dead link or latched faults."
        if self.backend.get("payload_link", "link_ok") < 0.5:
            raise HardwareFault("payload link down (no telemetry)")
        flags = int(self.backend.get("payload_link", "fault_flags") or 0)
        if flags & ~1:            # bit 0 (rx timeout) is covered by link_ok above
            raise HardwareFault(f"hardware interface fault flags 0x{flags:x}")

    #Timing and abort
    def _check_abort(self):
        if self.abort.is_set():
            raise Aborted()

    def wait(self, seconds):
        "Interruptible wait (abort ends it immediately)."
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._check_abort()
            self.backend.poll(min(self.poll_s, max(0.0, end - time.monotonic())))
    #stopping

    def safe_state(self, stoppers):
        """Stop everything we have driven.  `stoppers` = list of (component, interface, stop_value).Ignores the abort flag: this is what runs BECAUSE of an abort."""
        for comp, iface, value in stoppers:
            if (comp, iface) in self._driven:
                self._last[(comp, iface)] = value
                self.backend.send(comp, iface, float(value))

    def halt_motion(self, joints):
        "Retarget moving joints to where they are now (a stop here for position-controlled axes)."
        for comp, iface in joints:
            v = self.backend.get(comp, iface)
            if (comp, iface) in self._driven and not math.isnan(v):
                self._last[(comp, iface)] = v
                self.backend.send(comp, iface, v)


# RosBackend -- talks to the ros2_control controllers.  UNVERIFIED on Jazzy (no ROS here):
# topic names and message types follow the controller documentation; check them with
#     ros2 topic list ;  ros2 topic info -v <topic>
JOINT_GROUPS = {
    # controller name           : (command interface, ordered joint list)  -- order must match controllers.yaml
    "main_position_controller": ("position", ["vps_screw_rotation", "drill_lift_screw_rotation", "carousel_rotation", "soil_sampler_rotation"]),
    "drill_velocity_controller": ("velocity", ["drill_tool_rotation"]),
}
IO_CONTROLLER = "payload_io_controller"


def fill_group_command(joints, updates, last_cmd, state):
    """Build the full vector a joint-group controller needs, WITHOUT disturbing joints we did not mean to move.

    A group controller takes one array holding every joint.  For a joint we are not changing we must
    resend its previous command (not its current measured position: that would cancel a move in progress).
    A joint never commanded yet gets its current state, which the hardware interface recognises as
    "already there" and does not transmit.
    """
    out = []
    for j in joints:
        if j in updates:
            out.append(updates[j])
        elif j in last_cmd:
            out.append(last_cmd[j])
        else:
            s = state.get(j, float("nan"))
            out.append(0.0 if math.isnan(s) else s)
    return out


class RosBackend:
    def __init__(self, node):
        from std_msgs.msg import Float64MultiArray
        from sensor_msgs.msg import JointState
        from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
        self._F, self._DIGV, self._IV = Float64MultiArray, DynamicInterfaceGroupValues, InterfaceValue
        self.node = node
        self.state = {}                       # (component, interface) -> latest value
        self._group_cmd = {c: {} for c in JOINT_GROUPS}
        self._group_pub = {c: node.create_publisher(Float64MultiArray, f"/{c}/commands", 10) for c in JOINT_GROUPS}
        self._io_pub = node.create_publisher(DynamicInterfaceGroupValues, f"/{IO_CONTROLLER}/commands", 10)
        node.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        node.create_subscription(DynamicInterfaceGroupValues, f"/{IO_CONTROLLER}/gpio_states", self._on_gpio, 10)

    def _on_joints(self, msg):
        for i, n in enumerate(msg.name):
            if i < len(msg.position): self.state[(n, "position")] = msg.position[i]
            if i < len(msg.velocity): self.state[(n, "velocity")] = msg.velocity[i]

    def _on_gpio(self, msg):
        for group, vals in zip(msg.interface_groups, msg.interface_values):
            for name, v in zip(vals.interface_names, vals.values):
                self.state[(group, name)] = v

    def send(self, comp, iface, value):
        for ctrl, (kind, joints) in JOINT_GROUPS.items():
            if comp in joints and iface == kind:
                st = {j: self.state.get((j, kind), float("nan")) for j in joints}
                self._group_cmd[ctrl][comp] = value
                msg = self._F()
                msg.data = fill_group_command(joints, {comp: value}, {k: v for k, v in self._group_cmd[ctrl].items() if k != comp}, st)
                self._group_pub[ctrl].publish(msg)
                return
        msg = self._DIGV()
        msg.interface_groups = [comp]
        iv = self._IV()
        iv.interface_names = [iface]
        iv.values = [float(value)]
        msg.interface_values = [iv]
        self._io_pub.publish(msg)

    def get(self, comp, iface):
        return self.state.get((comp, iface), float("nan"))

    def poll(self, dt):
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=dt)
