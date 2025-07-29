# --- RSCP quick-send helper (drop-in) ---------------------------------------
import json
import atexit
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

__rscp_qs_node = None
__rscp_qs_pub = None

def _rscp_qs_ensure_node():
    global __rscp_qs_node, __rscp_qs_pub
    if not rclpy.ok():
        rclpy.init()
    if __rscp_qs_node is None:
        __rscp_qs_node = rclpy.create_node('rscp_quick_sender')
        __rscp_qs_pub = __rscp_qs_node.create_publisher(String, '/send_response', 10)
        atexit.register(_rscp_qs_shutdown)

def _rscp_qs_shutdown():
    global __rscp_qs_node, __rscp_qs_pub
    if __rscp_qs_node is not None:
        __rscp_qs_node.destroy_node()
        __rscp_qs_node = None
    if rclpy.ok():
        rclpy.shutdown()

def rscp_send(type_, **fields):
    """
    Publish a request that the RSCP Response Node send the given response type.
    type_ examples: 'task_finished', 'ack', 'gps', 'distance', 'message', 'rover_status'.
    Extra keyword args become JSON fields (see mapping below).
    """
    _rscp_qs_ensure_node()
    msg = String()
    payload = {'type': type_}
    payload.update(fields)
    msg.data = json.dumps(payload)
    __rscp_qs_pub.publish(msg)
    # give DDS a chance to send; non-blocking spin_once is usually enough
    rclpy.spin_once(__rscp_qs_node, timeout_sec=0.0)
# --------------------------------------------------------