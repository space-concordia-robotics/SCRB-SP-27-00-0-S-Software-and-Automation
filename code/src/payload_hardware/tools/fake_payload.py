#!/usr/bin/env python3
"""
fake_payload.py -- a pretend MAIN board for bench testing the hardware interface on vcan0.

    sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
    python3 tools/fake_payload.py vcan0          # then:  ros2 launch payload_bringup bringup.launch.py hardware:=can

It does what the firmware will do, as far as the CAN table defines it:
  * listens for command frames (destination = payload) and moves its simulated devices,
  * sends the telemetry messages at 20 Hz (destination = Jetson) and CONFIGURATION at 1 Hz,
  * holds positions by itself - nothing has to keep re-sending a command.
It uses can_table.py, an independent implementation of the bit layout.

Speeds are taken from the URDF limits (VPS 6.15 mm/s, drill platform 4.67 mm/s).
"""
import socket, struct, sys, time, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import can_table as ct

CAN_EFF_FLAG = 0x80000000
FMT = "=IB3x8s"                 # struct can_frame: id, dlc, pad, data[8]


class FakePayload:
    def __init__(self):
        self.vps_mm = 0.0; self.vps_target = 0.0       # box travel from home, mm
        self.drill_mm = 0.0; self.drill_target = 0.0   # drill depth, mm
        self.sas_deg = 0.0; self.carousel = 0
        self.drill_rpm = 0; self.plug = 0
        self.pump = {0: 0, 1: 0}; self.ultrasonic = 0; self.led = {0: 0, 1: 0}
        self.mission = 0; self.killed = False
        self.erc = dict(drill_dist=0, mass=0, rock=0, soil_mass=0, temp=25, spool=0)
        self.log = []

    # ---- a command arrives ----
    def handle(self, word):
        dest, name, f = ct.decode(word)
        if dest != 0 or name is None:
            return
        self.log.append((name, f))
        if name == "KILL": self.killed = True
        elif name == "VPS_HOME": self.vps_target = 0.0
        elif name == "VPS_MOVE": self.vps_target = float(f["target_displacement"])
        elif name == "SSS_POSITION": self.sas_deg = float(f["target_angle"])
        elif name == "URC_PUMP_RUN": self.pump[f["selector"]] = f["power"]
        elif name == "ULTRASONIC_RUN": self.ultrasonic = f["on"]
        elif name == "CAROUSEL_POSITION": self.carousel = f["cuvette"]
        elif name == "LED_SET": self.led[f["selector"]] = f["on"]
        elif name == "URC_DSS_DRILL_SET": self.drill_rpm = f["rpm"]
        elif name == "URC_DSS_HOME": self.drill_target = 0.0
        elif name == "URC_DSS_MOVE": self.drill_target = float(f["target_displacement"])
        elif name == "URC_DSS_PLUG_SET": self.plug = 1 if f["angle"] > 0 else 0
        elif name == "SET_MISSION": self.mission = f["mission"]
        # (ERC commands are accepted and ignored in this simple model)

    # ---- time passes ----
    def step(self, dt):
        def move(cur, tgt, speed):
            d = tgt - cur
            return tgt if abs(d) <= speed * dt else cur + speed * dt * (1 if d > 0 else -1)
        if not self.killed:
            self.vps_mm = move(self.vps_mm, self.vps_target, 6.15)
            self.drill_mm = move(self.drill_mm, self.drill_target, 4.67)

    # ---- telemetry ----
    def telemetry(self):
        c = lambda v, hi: max(0, min(int(round(v)), hi))
        w = {}
        w["TELEM_URC_GENERAL"] = ct.encode("TELEM_URC_GENERAL", 1, dict(
            distance_from_ground=c(300 - self.vps_mm, 1000), vps_displacement=c(self.vps_mm, 1000)))
        w["TELEM_URC_ANALYZE"] = ct.encode("TELEM_URC_ANALYZE", 1, dict(
            vps_rpm=0, sas_angle=c(self.sas_deg, 300), pump1_status=int(self.pump[0] != 0), pump2_status=int(self.pump[1] != 0),
            solenoid1=0, solenoid2=0, ultrasonic=self.ultrasonic, lid_servo=0, carousel_pos=self.carousel,
            led_405=self.led[0], led_445=self.led[1], current_mission=self.mission, photodiode=1234))
        w["TELEM_URC_DEEP_SAMPLE"] = ct.encode("TELEM_URC_DEEP_SAMPLE", 1, dict(
            vps_rpm=0, drill_disp=c(self.drill_mm, 127), drill_vert_rpm=0, drill_rot_rpm=c(self.drill_rpm, 100), plug_state=self.plug))
        # ERC telemetry is sent at all times; here it is zeros (the Jetson reads it as NaN in a URC run)
        w["TELEM_ERC_DRILL"] = ct.encode("TELEM_ERC_DRILL", 1, dict(drill_dist=0, drill_vert_rpm=0, drill_rot_rpm=0, load_cell_mass=0, deflector_state=0))
        w["TELEM_ERC_ROCK"] = ct.encode("TELEM_ERC_ROCK", 1, dict(rock_mass=0))
        w["TELEM_ERC_SOIL"] = ct.encode("TELEM_ERC_SOIL", 1, dict(soil_mass=0, temperature=25, servo_state=0))
        w["TELEM_ERC_SPOOL"] = ct.encode("TELEM_ERC_SPOOL", 1, dict(spool_angle=0, spool_rpm=0))
        w["TELEM_ERC_SPOOL_POS"] = ct.encode("TELEM_ERC_SPOOL_POS", 1, dict(spool_angle_now=0))
        return w

    def configuration(self):
        return ct.encode("CONFIGURATION", 1, {})


def main(ifname):
    s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((ifname,))
    s.setblocking(False)
    fp = FakePayload()
    print(f"fake payload on {ifname}; Ctrl-C to stop")
    last = t_tel = t_cfg = time.time()
    while True:
        now = time.time()
        try:
            while True:
                raw = s.recv(16)
                can_id, dlc, data = struct.unpack(FMT, raw)
                if (can_id & CAN_EFF_FLAG) and (can_id & 0x1FFFFFFF) == ct.ARB_ID and dlc == 8:
                    fp.handle(ct.from_bytes(data))
        except BlockingIOError:
            pass
        fp.step(now - last); last = now
        if now - t_tel >= 0.05:
            t_tel = now
            for word in fp.telemetry().values():
                s.send(struct.pack(FMT, ct.ARB_ID | CAN_EFF_FLAG, 8, ct.to_bytes(word)))
        if now - t_cfg >= 1.0:
            t_cfg = now
            s.send(struct.pack(FMT, ct.ARB_ID | CAN_EFF_FLAG, 8, ct.to_bytes(fp.configuration())))
        time.sleep(0.002)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "vcan0")
