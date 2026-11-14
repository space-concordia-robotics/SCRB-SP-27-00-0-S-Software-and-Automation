# SCRB-SP-27-00-0-S-Software-and-Automation
Repository containing the Ros2 packages for the control and orchestration of the URC and ERC science payload. The code for the data analysis needed for the science experiments are also found in this repository.

---
# Software Architecture
<img width="794" height="906" alt="image" src="https://github.com/user-attachments/assets/777b309b-fc8b-4396-a9c7-240a15f30273" />


# SCRB Science Payload Software (v3) 
#### If you got any question, or demo on your part is not working, let me know on google chat.
 
Software for the reusable science payload for URC 2027 and ERC 2027.
Stack: Jetson, ROS 2 Jazzy, ros2_control, CAN bus to the MAIN board.
 
```
GUI -> Mission node -> ros2_control controllers -> Hardware interface -> CAN -> MAIN board -> URC/ERC boards
```
 
> **Status in one line:** the CAN logic and the URC Take Deep Sample mission run and are tested
> *without ROS and without hardware*. Nothing has yet been run on real ROS 2 controllers, a real
> CAN bus, or real firmware.
 
## What the code can do today
 
| Part | What it does now | How it has been exercised |
|---|---|---|
| CAN table generator | Reads `CAN_Table.xlsx`, the URDF and `can_signals.yaml`, cross-checks them, and generates the C++ message table | Runs on a laptop |
| Hardware interface core (C++) | Packs commands into CAN frames and unpacks telemetry. Sends a command only when it changes. Rejects out-of-range values. Confirms a move against telemetry, retries twice, then latches a fault. Marks stale or other-competition telemetry as NaN. Sends stop commands on shutdown | Unit tests with a fake transmitter |
| Hardware interface ROS plugin | Wraps the core as a ros2_control `SystemInterface` (`payload_hardware/PayloadCanSystem`) | **Not yet run** |
| ros2_control setup | xacro, `controllers.yaml` (4 standard controllers), launch file, with `mock` or `can` hardware | **Not yet run** |
| URC mission layer (Python) | Runs the 10-step Take Deep Sample mission from a JSON step file: unit conversion from the URDF, travel-limit checks, send-once and confirm, abort, and rollback of reversible steps in reverse order | Unit tests against a simulated payload |
| Fake MAIN board | Pretends to be the payload on a virtual CAN bus: moves the VPS and drill at URDF speeds and sends telemetry | Logic test only |
 
### Not written yet
 
- ROS wiring of the mission node (GUI action server, operator prompt, progress and telemetry to the GUI)
- URC Analyze Site and all other URC missions
- All ERC mission modules
 

 
## Repository layout
 
```
src/
  payload_hardware/   CAN hardware interface (C++), generator, fake board, tests
  payload_bringup/    ros2_control xacro, controllers.yaml, launch file, contract check
  payload_mission/    URC mission layer (Python), step files, tests
```
 
## Try it out
 
Run everything from the repository root (the folder that contains `src/`).
 
### 1. Without ROS (works on any Linux laptop)
 
Needs: `python3`, `cmake`, a C++17 compiler, and `pip install openpyxl pyyaml`.
 
```bash
# a) Cross-check the CAN table, the yaml and the URDF, and regenerate the C++ table
python3 src/payload_hardware/scripts/generate_can_table.py
#    expected last lines:  messages 40, state bindings 31, command bindings 31
#                          problems: 0, notes: 19
 
# b) Check that every interface in the ros2_control xacro is backed by a CAN message
python3 src/payload_bringup/scripts/check_contract.py
#    expected last line:   contract OK
 
# c) Build and run the C++ tests of the hardware interface core
cmake -S src/payload_hardware -B /tmp/hwbuild
cmake --build /tmp/hwbuild
ctest --test-dir /tmp/hwbuild --output-on-failure
#    expected:             100% tests passed   (add -V to see "328 checks passed, 0 failed")
 
# d) Run Take Deep Sample against the simulated payload (takes about 45 s, mostly waiting on timeouts)
python3 src/payload_mission/tests/test_urc.py
#    expected last line:   mission layer OK (32 checks)
 
# e) Test the fake board's logic
python3 src/payload_hardware/test/test_fake_payload.py
#    expected:             fake payload logic OK
```
 
Notes:
- If ROS 2 is sourced in the shell, do **not** have a Python virtual environment active (`deactivate` first),
  or CMake fails with `No module named 'catkin_pkg'`.
- Run the C++ tests through `ctest`. Running `test_core` by hand from another folder reports one
  failure because it cannot find `test/vectors.txt`.
To watch the mission run step by step:
 
```bash
cd src/payload_mission
python3 - <<'EOF'
from payload_mission.units import Units
from payload_mission.fake_backend import FakeBackend
from payload_mission.mission_node_urc import UrcMission
 
urdf = "config/payload_description.urdf"
backend = FakeBackend(Units(urdf), speedup=400)
# backend.stuck.add("drill_lift_screw_rotation")   # uncomment to see a FAULT and the rollback
mission = UrcMission(backend, urdf, "missions", on_progress=lambda step, status: print(step, status))
print(mission.run("urc_take_deep_sample"))
EOF
```
 
### 2. Virtual CAN bus, still without ROS (not yet verified)
 
Needs `sudo apt install can-utils`.
 
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
 
python3 src/payload_hardware/tools/fake_payload.py vcan0     # terminal A: the fake MAIN board
candump vcan0                                                # terminal B: watch every frame
 
# terminal C: send "move the drill to 100 mm" by hand (message URC_DSS_MOVE)
cansend vcan0 00020000#26C8000000000000
```
 
What to look for: telemetry frames from the fake board in `candump`, and the drill displacement
in them changing after the `cansend`.
 
### 3. ROS 2 Jazzy (not yet verified)
 
```bash
deactivate 2>/dev/null                 # no Python venv
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install         # from the repository root, not from src/
source install/setup.bash
 
# a) mock hardware: checks the URDF, xacro, controllers and launch file
ros2 launch payload_bringup bringup.launch.py hardware:=mock
ros2 control list_controllers                # second terminal: expect 4 controllers, all "active"
ros2 control list_hardware_interfaces
 
# b) real plugin on the virtual bus, with fake_payload.py and candump running as in section 2
ros2 launch payload_bringup bringup.launch.py hardware:=can can_interface:=vcan0 competition:=urc
 
# c) command the VPS screw and watch candump / RViz
ros2 topic pub --once /main_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [-143.0, 0.0, 0.0, 0.0]}"          # order: vps screw, drill lift, carousel, soil sampler
```
 
For 3D viewing in RViz, add the mechanical team's `payload_description_v46` package to `src/`
(it provides the meshes), set Fixed Frame to `telescopic_frame` and add a RobotModel display on
`/robot_description`.
 
On a real CAN interface, bring it up with `sudo src/payload_hardware/tools/setup_can.sh can0 <bitrate>`
(sets `restart-ms 100` and `txqueuelen 100`).
 
## After changing an input
 
| You changed | Re-run |
|---|---|
| `CAN_Table.xlsx` or `can_signals.yaml` | `generate_can_table.py`, `check_contract.py`, the C++ tests |
| The URDF | Replace **all three** copies, then `generate_can_table.py`, `check_contract.py`, `test_urc.py` |
| The ros2_control xacro | `check_contract.py`, then `src/payload_bringup/scripts/make_controllers_yaml.py` |
 
Do not edit `can_table_generated.hpp` or `can_table.json` by hand. They are generated.
 
## Behaviour worth knowing
 
- The Jetson sends a command when the target **changes**. The boards hold position themselves.
- NaN means "no value". It is normal only for the competition that is not active.
- Out-of-range requests are rejected, not clamped.
- Limit switches and motor protection belong to the firmware. The software checks are a second layer.
## More
 
- `src/payload_hardware/docs/CAN_TABLE_REPORT.md`: message list and open assumptions
- `Payload_Software_Explained.pdf` (shared separately, not in this repository): how it all works, from first principles
 
