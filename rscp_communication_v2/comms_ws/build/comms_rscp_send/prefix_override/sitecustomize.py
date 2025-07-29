import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/legendary/eric_anatolian/rscp_communication/comms_ws/install/comms_rscp_send'
