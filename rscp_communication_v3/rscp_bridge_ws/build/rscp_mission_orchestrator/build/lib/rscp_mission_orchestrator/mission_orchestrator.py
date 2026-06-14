#!/usr/bin/env python3
"""
mission_orchestrator.py
=======================

The "brain" node for the rover. It sits between the RSCP bridge
(rscp_bridge_node) and the subsystem worker nodes, and turns decoded RSCP
commands into actions performed by the rover.

Data flow
---------
    rscp_bridge_node                     mission_orchestrator
    ----------------                     --------------------
    recv_rscp (JSON command)  --------->  _on_command() dispatch
                                              |
                                              |  service call / action goal
                                              v
                                          arm_control_node   (std_srvs/SetBool)
                                          search_area_node   (SearchArea.action)
                                          navigation_node    (NavigateToGPS.action -> Nav2)
                                          exploration_node   (Explore.action)
                                              |
                              <---------  results / feedback
    send_rscp (JSON response) <---------  _send_*() helpers

Translation contract (matches the bridge's JSON schema)
-------------------------------------------------------
Incoming on rover_missions/recv_rscp:
    {"type": "set_stage", "stage": int}
    {"type": "arm_disarm", "arm": bool}
    {"type": "search_area", "lat": float, "lon": float, "rad": float}
    {"type": "navigate_to_gps", "lat": float, "lon": float}
    {"type": "start_exploration"}

Outgoing on rover_missions/send_rscp:
    {"type": "acknowledge"}                       # sent on command accept/success
    {"type": "gps", "lat": float, "lon": float}   # result of a search
    {"type": "distance", "distance": float}       # exploration telemetry
    {"type": "task_completed"}                    # task finished

Acknowledge timing (important for RSCP, which has no correlation field):
    * set_stage      -> ack immediately after state update
    * arm_disarm     -> ack after the arm service reports success
    * long-running   -> ack as soon as the action goal is ACCEPTED, then
                        results/telemetry follow, then task_completed.

Requires the custom action interfaces in `rscp_mission_interfaces`:
    action/SearchArea.action
    action/NavigateToGPS.action
    action/Explore.action
(See the accompanying setup instructions for these definitions.)

Dependencies:  rclpy, std_msgs, std_srvs, rscp_mission_interfaces
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from std_srvs.srv import SetBool

from rscp_mission_interfaces.action import SearchArea, NavigateToGPS, Explore


class MissionOrchestrator(Node):
    """Translates RSCP commands into rover actions and reports results back."""

    def __init__(self):
        super().__init__("mission_orchestrator")

        # Reentrant group so action result/feedback callbacks can run while the
        # command subscription is busy (required with a MultiThreadedExecutor).
        self._cb = ReentrantCallbackGroup()

        # ---- Mission state --------------------------------------------------
        self._current_stage = 0
        self._armed = False
        self._active_goal_handle = None  # the one in-flight long-running task

        # ---- Bridge interface ----------------------------------------------
        self._sub = self.create_subscription(
            String, "rover_missions/recv_rscp", self._on_command, 10,
            callback_group=self._cb,
        )
        self._pub = self.create_publisher(
            String, "rover_missions/send_rscp", 50,
        )

        # ---- Downstream subsystem clients ----------------------------------
        self._arm_client = self.create_client(
            SetBool, "arm_disarm", callback_group=self._cb,
        )
        self._search_client = ActionClient(
            self, SearchArea, "search_area", callback_group=self._cb,
        )
        self._nav_client = ActionClient(
            self, NavigateToGPS, "navigate_to_gps", callback_group=self._cb,
        )
        self._explore_client = ActionClient(
            self, Explore, "explore", callback_group=self._cb,
        )

        # Dispatch table: command type -> handler
        self._handlers = {
            "set_stage": self._handle_set_stage,
            "arm_disarm": self._handle_arm_disarm,
            "search_area": self._handle_search_area,
            "navigate_to_gps": self._handle_navigate_to_gps,
            "start_exploration": self._handle_start_exploration,
        }

        self.get_logger().info("Mission orchestrator ready.")

    # ========================================================================
    # Command intake / dispatch
    # ========================================================================
    def _on_command(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Ignoring non-JSON command: {msg.data!r}")
            return
        if not isinstance(data, dict):
            self.get_logger().warn("Command JSON must be an object.")
            return

        ctype = data.get("type")
        handler = self._handlers.get(ctype)
        if handler is None:
            self.get_logger().warn(f"No handler for command type: {ctype!r}")
            return

        self.get_logger().info(f"Command received: {data}")
        try:
            handler(data)
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f"Malformed '{ctype}' command {data!r}: {exc}")

    # ========================================================================
    # Handlers
    # ========================================================================
    def _handle_set_stage(self, data):
        stage = int(data["stage"])
        # A new stage supersedes any running task.
        self._cancel_active_goal()
        self._current_stage = stage
        self.get_logger().info(f"Stage set to {stage}.")
        self._send_ack()

    def _handle_arm_disarm(self, data):
        arm = bool(data["arm"])
        if not self._arm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("arm_disarm service unavailable; cannot ack.")
            return
        req = SetBool.Request()
        req.data = arm
        future = self._arm_client.call_async(req)
        future.add_done_callback(lambda f: self._on_arm_done(f, arm))

    def _on_arm_done(self, future, arm):
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"arm_disarm service call failed: {exc}")
            return
        if resp.success:
            self._armed = arm
            self.get_logger().info(f"Rover {'armed' if arm else 'disarmed'}.")
            self._send_ack()
        else:
            self.get_logger().warn(f"arm/disarm rejected: {resp.message}")

    def _handle_search_area(self, data):
        if not self._begin_action(self._search_client, "search_area"):
            return
        goal = SearchArea.Goal()
        goal.lat = float(data["lat"])
        goal.lon = float(data["lon"])
        goal.radius = float(data["rad"])
        send_future = self._search_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_search_goal_response)

    def _on_search_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("search_area goal was rejected.")
            return
        self._active_goal_handle = handle
        self._send_ack()  # accepted -> acknowledge to host
        handle.get_result_async().add_done_callback(self._on_search_result)

    def _on_search_result(self, future):
        result = future.result().result
        if getattr(result, "success", True):
            self._send_gps(result.found_lat, result.found_lon)
        self._send_task_completed()
        self._active_goal_handle = None

    def _handle_navigate_to_gps(self, data):
        if not self._begin_action(self._nav_client, "navigate_to_gps"):
            return
        goal = NavigateToGPS.Goal()
        goal.lat = float(data["lat"])
        goal.lon = float(data["lon"])
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_nav_goal_response)

    def _on_nav_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("navigate_to_gps goal was rejected.")
            return
        self._active_goal_handle = handle
        self._send_ack()
        handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        # Navigation produces no coordinate of its own, just completion.
        self._send_task_completed()
        self._active_goal_handle = None

    def _handle_start_exploration(self, data):
        if not self._begin_action(self._explore_client, "explore"):
            return
        goal = Explore.Goal()
        send_future = self._explore_client.send_goal_async(
            goal, feedback_callback=self._on_explore_feedback
        )
        send_future.add_done_callback(self._on_explore_goal_response)

    def _on_explore_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("explore goal was rejected.")
            return
        self._active_goal_handle = handle
        self._send_ack()
        handle.get_result_async().add_done_callback(self._on_explore_result)

    def _on_explore_feedback(self, feedback_msg):
        # Stream distance telemetry to the host as the rover traverses.
        self._send_distance(feedback_msg.feedback.distance)

    def _on_explore_result(self, future):
        self._send_task_completed()
        self._active_goal_handle = None

    # ========================================================================
    # Helpers
    # ========================================================================
    def _begin_action(self, client: ActionClient, name: str) -> bool:
        """Cancel any running task and confirm the target server is available."""
        self._cancel_active_goal()
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"'{name}' action server unavailable.")
            return False
        return True

    def _cancel_active_goal(self):
        """Best-effort cancel of the currently running long task, if any."""
        if self._active_goal_handle is not None:
            self.get_logger().info("Cancelling previous in-flight task.")
            try:
                self._active_goal_handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"Cancel request failed: {exc}")
            self._active_goal_handle = None

    def _send_response(self, payload: dict):
        out = String()
        out.data = json.dumps(payload)
        self._pub.publish(out)
        self.get_logger().info(f"-> bridge: {out.data}")

    def _send_ack(self):
        self._send_response({"type": "acknowledge"})

    def _send_gps(self, lat: float, lon: float):
        self._send_response({"type": "gps", "lat": float(lat), "lon": float(lon)})

    def _send_distance(self, distance: float):
        self._send_response({"type": "distance", "distance": float(distance)})

    def _send_task_completed(self):
        self._send_response({"type": "task_completed"})


def main(args=None):
    rclpy.init(args=args)
    node = MissionOrchestrator()
    # MultiThreadedExecutor is required so action callbacks and the command
    # subscription can run concurrently without deadlocking.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
