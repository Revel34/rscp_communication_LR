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

Worker fault tolerance
----------------------
Each long-running task is protected by a watchdog timer. If a worker node
hangs (never returns a result) or crashes (result future raises), the
orchestrator:
    1. logs the failure,
    2. attempts to cancel the stuck goal so the worker can recover, and
    3. clears its internal state so the next command works normally.
By design the orchestrator stays SILENT to the host on failure/timeout — the
Host Module's own timeout logic remains the single source of truth, so no
false `task_completed` is ever sent. Per-task timeouts are ROS2 parameters.

Requires the custom action interfaces in `rscp_mission_interfaces`:
    action/SearchArea.action
    action/NavigateToGPS.action
    action/Explore.action

Dependencies:  rclpy, std_msgs, std_srvs, rscp_mission_interfaces
"""

import json
import threading

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

        # ---- Parameters: per-task timeouts (seconds) -----------------------
        # 0 disables the watchdog for that task. Tune to your mission limits.
        self.declare_parameter("search_area_timeout_sec", 120.0)
        self.declare_parameter("navigate_to_gps_timeout_sec", 180.0)
        self.declare_parameter("explore_timeout_sec", 300.0)
        self.declare_parameter("arm_service_timeout_sec", 5.0)

        # ---- Mission state --------------------------------------------------
        self._current_stage = 0
        self._armed = False
        self._state_lock = threading.Lock()
        self._active_goal_handle = None   # the one in-flight long-running task
        self._active_task_name = None     # human-readable name of that task
        self._watchdog = None             # rclpy timer that fires on timeout

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
        with self._state_lock:
            self._current_stage = stage
        self.get_logger().info(f"Stage set to {stage}.")
        self._send_ack()

    def _handle_arm_disarm(self, data):
        arm = bool(data["arm"])
        timeout = float(self.get_parameter("arm_service_timeout_sec").value)
        if not self._arm_client.wait_for_service(timeout_sec=timeout):
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
            with self._state_lock:
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
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"search_area goal send failed: {exc}")
            return
        if not handle.accepted:
            self.get_logger().warn("search_area goal was rejected.")
            return
        self._register_active_goal(
            handle, "search_area",
            float(self.get_parameter("search_area_timeout_sec").value),
        )
        self._send_ack()  # accepted -> acknowledge to host
        handle.get_result_async().add_done_callback(self._on_search_result)

    def _on_search_result(self, future):
        if not self._consume_result_guarded(future, "search_area"):
            return
        result = future.result().result
        if getattr(result, "success", True):
            self._send_gps(result.found_lat, result.found_lon)
        self._send_task_completed()
        self._clear_active_goal()

    def _handle_navigate_to_gps(self, data):
        if not self._begin_action(self._nav_client, "navigate_to_gps"):
            return
        goal = NavigateToGPS.Goal()
        goal.lat = float(data["lat"])
        goal.lon = float(data["lon"])
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_nav_goal_response)

    def _on_nav_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"navigate_to_gps goal send failed: {exc}")
            return
        if not handle.accepted:
            self.get_logger().warn("navigate_to_gps goal was rejected.")
            return
        self._register_active_goal(
            handle, "navigate_to_gps",
            float(self.get_parameter("navigate_to_gps_timeout_sec").value),
        )
        self._send_ack()
        handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        if not self._consume_result_guarded(future, "navigate_to_gps"):
            return
        # Navigation produces no coordinate of its own, just completion.
        self._send_task_completed()
        self._clear_active_goal()

    def _handle_start_exploration(self, data):
        if not self._begin_action(self._explore_client, "explore"):
            return
        goal = Explore.Goal()
        send_future = self._explore_client.send_goal_async(
            goal, feedback_callback=self._on_explore_feedback
        )
        send_future.add_done_callback(self._on_explore_goal_response)

    def _on_explore_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"explore goal send failed: {exc}")
            return
        if not handle.accepted:
            self.get_logger().warn("explore goal was rejected.")
            return
        self._register_active_goal(
            handle, "explore",
            float(self.get_parameter("explore_timeout_sec").value),
        )
        self._send_ack()
        handle.get_result_async().add_done_callback(self._on_explore_result)

    def _on_explore_feedback(self, feedback_msg):
        # Stream distance telemetry to the host as the rover traverses.
        # Feedback also proves the worker is alive, so pet the watchdog.
        self._pet_watchdog()
        self._send_distance(feedback_msg.feedback.distance)

    def _on_explore_result(self, future):
        if not self._consume_result_guarded(future, "explore"):
            return
        self._send_task_completed()
        self._clear_active_goal()

    # ========================================================================
    # Active-task tracking + watchdog
    # ========================================================================
    def _register_active_goal(self, handle, name: str, timeout_sec: float):
        """Record the in-flight goal and arm a watchdog timer for it."""
        with self._state_lock:
            self._active_goal_handle = handle
            self._active_task_name = name
            self._cancel_watchdog_locked()
            if timeout_sec and timeout_sec > 0.0:
                # One-shot timer: fires once after timeout_sec if not cleared.
                self._watchdog = self.create_timer(
                    timeout_sec, self._on_watchdog_timeout,
                    callback_group=self._cb,
                )
        if timeout_sec and timeout_sec > 0.0:
            self.get_logger().info(
                f"Task '{name}' started; watchdog armed for {timeout_sec:.0f}s."
            )

    def _pet_watchdog(self):
        """Reset the watchdog countdown — called when a worker proves liveness
        via feedback. Restarts the one-shot timer from zero."""
        with self._state_lock:
            if self._watchdog is not None and self._active_task_name is not None:
                self._watchdog.reset()

    def _on_watchdog_timeout(self):
        """Fired when a task exceeds its timeout. Cancel it and reset state.
        Deliberately sends NOTHING to the host — the host's own timeout is
        authoritative, so we never emit a false task_completed."""
        with self._state_lock:
            name = self._active_task_name
            handle = self._active_goal_handle
            self._cancel_watchdog_locked()
        if name is None:
            return
        self.get_logger().error(
            f"WATCHDOG: task '{name}' did not respond in time. "
            f"Attempting to cancel it. (No message sent to host.)"
        )
        self._try_cancel(handle, name)
        with self._state_lock:
            self._active_goal_handle = None
            self._active_task_name = None

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

    def _consume_result_guarded(self, future, name: str) -> bool:
        """Return True if a result callback should proceed. Returns False if
        the worker crashed (future raised) or the task was already torn down
        by the watchdog (so we don't double-send or act on a dead task)."""
        with self._state_lock:
            still_active = (self._active_task_name == name)
        if not still_active:
            # Watchdog already handled this task as a timeout, or it was
            # superseded. Ignore the late/failed result.
            self.get_logger().warn(
                f"Ignoring result for '{name}': task already cleared "
                f"(timed out, cancelled, or superseded)."
            )
            return False
        try:
            future.result()  # raises if the worker crashed
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"Worker '{name}' crashed or returned an error: {exc}. "
                f"(No message sent to host.)"
            )
            self._clear_active_goal()
            return False
        return True

    def _cancel_active_goal(self):
        """Best-effort cancel of the currently running long task, if any."""
        with self._state_lock:
            handle = self._active_goal_handle
            name = self._active_task_name
            self._cancel_watchdog_locked()
            self._active_goal_handle = None
            self._active_task_name = None
        if handle is not None:
            self.get_logger().info(f"Cancelling previous task '{name}'.")
            self._try_cancel(handle, name)

    def _clear_active_goal(self):
        """Clear active-task state after a normal completion."""
        with self._state_lock:
            self._cancel_watchdog_locked()
            self._active_goal_handle = None
            self._active_task_name = None

    def _cancel_watchdog_locked(self):
        """Cancel + destroy the watchdog timer. Caller must hold _state_lock."""
        if self._watchdog is not None:
            try:
                self._watchdog.cancel()
                self.destroy_timer(self._watchdog)
            except Exception:  # noqa: BLE001
                pass
            self._watchdog = None

    def _try_cancel(self, handle, name):
        """Send a cancel request to a worker, swallowing any error."""
        if handle is None:
            return
        try:
            handle.cancel_goal_async()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Cancel request for '{name}' failed: {exc}")

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
    # MultiThreadedExecutor is required so action callbacks, the watchdog timer,
    # and the command subscription can run concurrently without deadlocking.
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
