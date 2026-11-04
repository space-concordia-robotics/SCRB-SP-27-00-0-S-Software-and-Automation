# SCRB-SP-27-00-0-S-Software-and-Automation
Repository containing the Ros2 packages for the control and orchestration of the URC and ERC science payload. The code for the data analysis needed for the science experiments are also found in this repository.

---
# Software Architecture
<img width="794" height="906" alt="image" src="https://github.com/user-attachments/assets/777b309b-fc8b-4396-a9c7-240a15f30273" />

## Trail
GUI -> Mission Node (URC / ERC modules) -> ros2_control -> C++ CAN hardware interface -> CAN -> MAIN board and URC/ERC boards
