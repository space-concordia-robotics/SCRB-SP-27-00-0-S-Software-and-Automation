import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/waqar/SCRB-SP-27-00-0-S-Software-and-Automation/install/payload_mission'
