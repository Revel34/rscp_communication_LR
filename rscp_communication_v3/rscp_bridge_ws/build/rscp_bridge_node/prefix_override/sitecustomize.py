import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/revel34/rscp_communication_LR/rscp_communication_v3/rscp_bridge_ws/install/rscp_bridge_node'
