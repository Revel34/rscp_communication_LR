# Mission Orchestrator

The mission orchestrator (`mission_orchestrator.py`) is the "brain" of the
rover's software stack for the **Anatolian Rover Challenge (ARC)**. It lives
in its own ROS2 package (`rscp_mission_orchestrator`) and sits between the
RSCP bridge node (which handles serial communication with the Host Module)
and the subsystem worker nodes (which control the rover's hardware).

```
Host Module
    |
    | RSCP / serial (COBS + Protobuf)
    v
[ rscp_bridge_node ]               (translator — communication layer)
    |
    | rover_missions/recv_rscp     (JSON commands)
    | rover_missions/send_rscp     (JSON responses)
    v
[ mission_orchestrator ]           (brain — this package)
    |
    |-- std_srvs/SetBool --------> [ arm_control_node  ]
    |-- SearchArea action --------> [ search_area_node  ]
    |-- NavigateToGPS action -----> [ navigation_node   ]
    |-- Explore action -----------> [ exploration_node  ]
```

The orchestrator does not drive motors, navigate, or read sensors. Its only
responsibilities are:

1. Receive decoded RSCP commands from the bridge.
2. Translate them into ROS2 action goals or service calls directed at the
   correct subsystem worker node.
3. Send acknowledgements, telemetry, and completion signals back to the host
   via the bridge at the correct time.

The actual rover behaviour is implemented by **worker nodes** built and owned
by subsystem teammates. This document explains exactly how to write them.

---

## Workspace layout

```
src/
├── rscp_bridge_node/              # bridge (translator) — separate package
├── rscp_mission_interfaces/       # shared action type definitions
└── rscp_mission_orchestrator/     # this package
    ├── rscp_mission_orchestrator/
    │   ├── __init__.py
    │   └── mission_orchestrator.py
    ├── launch/
    │   └── rover_bringup.launch.py
    ├── package.xml
    └── setup.py
```

---

## How the orchestrator works

### Command intake

The orchestrator subscribes to `rover_missions/recv_rscp`, the topic on which
the bridge publishes decoded RSCP commands as JSON strings. On every message
it reads the `"type"` field and dispatches to the matching internal handler:

| Command type | What the orchestrator does |
|---|---|
| `set_stage` | Updates current mission stage, cancels any running task, sends ack |
| `arm_disarm` | Calls the `arm_disarm` service, sends ack on success |
| `search_area` | Sends a `SearchArea` action goal, streams GPS result and completion |
| `navigate_to_gps` | Sends a `NavigateToGPS` action goal, reports completion |
| `start_exploration` | Sends an `Explore` action goal, streams distance feedback and completion |

### Acknowledge timing

RSCP acknowledgements carry no identifying information — the Host Module
infers what is being acknowledged purely from message order. The orchestrator
follows this strict ordering:

- **`set_stage`:** ack sent immediately after the stage variable is updated.
- **`arm_disarm`:** ack sent only after the arm service reports success.
- **Long-running tasks:** ack sent as soon as the action goal is **accepted**
  by the worker. Results and `task_completed` follow when the work is done.

### What the orchestrator sends back to the host

| Event | JSON sent to bridge |
|---|---|
| Command accepted / service success | `{"type": "acknowledge"}` |
| Search result found | `{"type": "gps", "lat": <float>, "lon": <float>}` |
| Exploration distance update | `{"type": "distance", "distance": <float>}` |
| Task finished | `{"type": "task_completed"}` |

---

## Running

```bash
ros2 launch rscp_mission_orchestrator rover_bringup.launch.py port:=/dev/ttyACM0
```

Or double-click `launch_rover_comms.sh` / the `RoverComms.desktop` icon.

With no worker nodes running yet the orchestrator will log
`"action server unavailable"` when commands arrive — this is expected and
correct until workers are connected.

---

## Writing worker nodes

A worker node is a Python (or C++)
ROS2 node that exposes a **service server** or **action server** under a
specific name. The orchestrator calls it automatically when the corresponding
RSCP command arrives.

### The four workers that need to be built

| Worker | What it does | Mechanism | Server name |
|---|---|---|---|
| Arm control | Arms or disarms motor power | Service (`std_srvs/SetBool`) | `arm_disarm` |
| Search area | Searches a GPS zone and finds a coordinate of interest | Action (`SearchArea`) | `search_area` |
| Navigation | Drives to a GPS coordinate using Nav2 | Action (`NavigateToGPS`) | `navigate_to_gps` |
| Exploration | Traverses the lava tube and streams distance measurements | Action (`Explore`) | `explore` |

### Two critical rules every worker must follow

**1. The server name must match exactly.**
Names are case-sensitive. A server named `Search_Area` or `searcharea`
will never connect to the orchestrator. Use exactly the names in the table
above.

**2. The action type must come from `rscp_mission_interfaces`.**
ROS2 verifies compatibility by hashing the `.action` definition. If a worker
defines its own version of `SearchArea.action` with different field names or
types, the connection will silently fail even if the server name matches.
Everyone must build their worker package with
`<depend>rscp_mission_interfaces</depend>` in `package.xml` and import from
`rscp_mission_interfaces.action`.

---

### Worker 1 — Arm control (Service)

The arm control worker is the simplest: it receives a boolean from the
orchestrator and arms or disarms the rover's motors. It uses a plain **ROS2
service** because the action is instantaneous and produces a simple
success/failure outcome.

The orchestrator calls it with `request.data = True` to arm and
`request.data = False` to disarm. It only sends `acknowledge` to the host if
`response.success` is `True`, so never set it to `True` if the hardware
actually failed.

```python
#!/usr/bin/env python3
"""
arm_control_node.py
-------------------
Service server: arm_disarm (std_srvs/SetBool)

Replace the placeholder logic with your actual motor enable/disable calls.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class ArmControlNode(Node):
    def __init__(self):
        super().__init__("arm_control_node")

        self._srv = self.create_service(
            SetBool, "arm_disarm", self._handle_arm_disarm
        )
        self.get_logger().info("Arm control service ready on 'arm_disarm'.")

    def _handle_arm_disarm(self, request, response):
        if request.data:
            self.get_logger().info("Arming rover...")
            # -----------------------------------------------------------------
            # YOUR CODE: enable motor power, e.g.:
            #   self._motor_pub.publish(EnableMsg(enabled=True))
            # -----------------------------------------------------------------
            armed_successfully = True   # replace with real outcome
            if armed_successfully:
                self.get_logger().info("Rover armed.")
                response.success = True
                response.message = "Armed"
            else:
                self.get_logger().error("Arm failed.")
                response.success = False
                response.message = "Arm failed"
        else:
            self.get_logger().info("Disarming rover...")
            # -----------------------------------------------------------------
            # YOUR CODE: disable motor power
            # -----------------------------------------------------------------
            response.success = True
            response.message = "Disarmed"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

**Test it without the orchestrator:**
```bash
ros2 service call /arm_disarm std_srvs/srv/SetBool "{data: true}"
ros2 service call /arm_disarm std_srvs/srv/SetBool "{data: false}"
```

---

### Worker 2 — Search area (Action)

The search area worker receives a GPS zone (center coordinate + radius),
physically drives the rover around searching for the point of interest (e.g.
the antenna signal peak in Stage 1, or the coldest surface point in Stage 2),
and returns the best coordinate it found.

This uses a **ROS2 action** because the search takes minutes, produces
intermediate position updates (feedback), and yields a GPS result at the end.

The `_execute` callback **must be `async def`**. A regular `def` blocks the
ROS2 executor for its entire duration, preventing feedback, cancellation, and
all other callbacks from running. Use `await asyncio.sleep()` in any loops
that wait for conditions.

```python
#!/usr/bin/env python3
"""
search_area_node.py
--------------------
Action server: search_area (rscp_mission_interfaces/SearchArea)

Goal    : lat, lon, radius  — center coordinate and search radius in metres
Feedback: current_lat, current_lon  — rover's position as it searches
Result  : found_lat, found_lon, success  — the coordinate of interest found

Replace the placeholder logic with your real navigation and sensor calls.
"""

import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from rscp_mission_interfaces.action import SearchArea


class SearchAreaNode(Node):
    def __init__(self):
        super().__init__("search_area_node")

        self._server = ActionServer(
            self,
            SearchArea,
            "search_area",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info("Search area action server ready on 'search_area'.")

    def _goal_callback(self, goal_request):
        """Accept all incoming goals."""
        self.get_logger().info(
            f"Goal received: center=({goal_request.lat}, {goal_request.lon}), "
            f"radius={goal_request.radius} m"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Allow the orchestrator to cancel mid-search (e.g. on new set_stage)."""
        self.get_logger().info("Search area cancel requested.")
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle):
        lat    = goal_handle.request.lat
        lon    = goal_handle.request.lon
        radius = goal_handle.request.radius

        self.get_logger().info(f"Starting search: center=({lat}, {lon}), radius={radius} m")

        feedback = SearchArea.Feedback()

        # ------------------------------------------------------------------
        # YOUR CODE: drive a search pattern within the given radius.
        #
        # Key points:
        #   - Check goal_handle.is_cancel_requested in your loop and stop
        #     cleanly if True. Return goal_handle.canceled() in that case.
        #   - Call goal_handle.publish_feedback(feedback) as the rover moves
        #     so the orchestrator can see current position.
        #   - Track the best coordinate found (peak RSSI, coldest temp, etc.)
        #
        # Example loop structure (replace with your real implementation):
        # ------------------------------------------------------------------
        best_lat = lat
        best_lon = lon

        for step in range(10):      # replace with your actual search steps
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Search cancelled.")
                goal_handle.canceled()
                return SearchArea.Result()

            # Simulate moving to a new position — replace with real nav call
            current_lat = lat + step * 0.00001
            current_lon = lon + step * 0.00001

            # Publish current position as feedback
            feedback.current_lat = current_lat
            feedback.current_lon = current_lon
            goal_handle.publish_feedback(feedback)

            # Update best coordinate based on sensor reading
            # e.g. if signal_strength > best_signal: best_lat, best_lon = ...
            best_lat = current_lat
            best_lon = current_lon

            await asyncio.sleep(0.1)    # yield to executor; replace with real wait

        # ------------------------------------------------------------------
        # Return the best coordinate found.
        # ------------------------------------------------------------------
        result = SearchArea.Result()
        result.found_lat = best_lat
        result.found_lon = best_lon
        result.success   = True
        goal_handle.succeed()

        self.get_logger().info(
            f"Search complete. Best coordinate: ({result.found_lat}, {result.found_lon})"
        )
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SearchAreaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

**Test it without the orchestrator:**
```bash
ros2 action send_goal /search_area rscp_mission_interfaces/action/SearchArea \
  "{lat: 39.9, lon: 32.8, radius: 25.0}"
```

---

### Worker 3 — Navigation (Action)

The navigation worker receives a GPS coordinate and drives the rover to it.
Internally it must convert the GPS coordinate into the rover's local map
frame (since Nav2 works in `map`/`odom` frames, not GPS) and then send a
goal to Nav2's `navigate_to_pose` action server.

The recommended way to do the GPS-to-map conversion is with
`navsat_transform_node` from the `robot_localization` package, which fuses
GPS and IMU data and publishes a continuous `map→odom` transform.

```python
#!/usr/bin/env python3
"""
navigation_node.py
------------------
Action server: navigate_to_gps (rscp_mission_interfaces/NavigateToGPS)

Goal    : lat, lon  — the GPS coordinate to drive to
Feedback: distance_remaining  — metres left to the goal
Result  : success

This node converts the GPS coordinate to a map-frame pose and forwards it
to Nav2's navigate_to_pose action server.

Replace the coordinate conversion placeholder with your real implementation
(e.g. using robot_localization's navsat_transform_node output).
"""

import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse

from rscp_mission_interfaces.action import NavigateToGPS
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation_node")

        # Action server exposed to the orchestrator
        self._server = ActionServer(
            self,
            NavigateToGPS,
            "navigate_to_gps",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        # Internal client to Nav2
        self._nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.get_logger().info("Navigation action server ready on 'navigate_to_gps'.")

    def _goal_callback(self, goal_request):
        self.get_logger().info(
            f"Navigation goal received: ({goal_request.lat}, {goal_request.lon})"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("Navigation cancel requested.")
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle):
        lat = goal_handle.request.lat
        lon = goal_handle.request.lon

        self.get_logger().info(f"Navigating to GPS ({lat}, {lon})...")

        # ------------------------------------------------------------------
        # YOUR CODE: convert GPS (lat, lon) to a map-frame PoseStamped.
        #
        # Option A: subscribe to /gps/fix and use robot_localization's
        #   navsat_transform_node to get a map-frame odometry, then compute
        #   the offset from current pose.
        #
        # Option B: use a UTM conversion library (utm, pyproj) to convert
        #   lat/lon to local ENU coordinates.
        #
        # Placeholder: passes the lat/lon directly as x/y — NOT correct for
        # a real deployment but lets the node compile and run.
        # ------------------------------------------------------------------
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = lon   # replace with real map-frame x
        pose.pose.position.y = lat   # replace with real map-frame y
        pose.pose.orientation.w = 1.0

        # Wait for Nav2 to be available
        if not self._nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 navigate_to_pose server unavailable.")
            goal_handle.abort()
            result = NavigateToGPS.Result()
            result.success = False
            return result

        # Send goal to Nav2
        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = pose
        send_future = self._nav2_client.send_goal_async(nav2_goal)
        await asyncio.ensure_future(send_future)
        nav2_handle = send_future.result()

        if not nav2_handle.accepted:
            self.get_logger().warn("Nav2 rejected the goal.")
            goal_handle.abort()
            result = NavigateToGPS.Result()
            result.success = False
            return result

        # Wait for Nav2 result, forwarding cancellation and feedback
        result_future = nav2_handle.get_result_async()
        feedback = NavigateToGPS.Feedback()

        while not result_future.done():
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Navigation cancelled. Cancelling Nav2...")
                await asyncio.ensure_future(nav2_handle.cancel_goal_async())
                goal_handle.canceled()
                return NavigateToGPS.Result()

            # ------------------------------------------------------------------
            # YOUR CODE: populate distance_remaining from your localisation
            # source, e.g. compute Euclidean distance to goal in map frame.
            # ------------------------------------------------------------------
            feedback.distance_remaining = 0.0   # replace with real value
            goal_handle.publish_feedback(feedback)
            await asyncio.sleep(0.5)

        nav2_result = result_future.result()
        result = NavigateToGPS.Result()
        result.success = True
        goal_handle.succeed()
        self.get_logger().info("Navigation complete.")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

**Test it without the orchestrator:**
```bash
ros2 action send_goal /navigate_to_gps rscp_mission_interfaces/action/NavigateToGPS \
  "{lat: 39.95, lon: 32.85}"
```

---

### Worker 4 — Exploration (Action)

The exploration worker handles lava tube traversal (Stage 3). It drives the
rover through the tube and continuously publishes distance measurements as
feedback. The orchestrator forwards each feedback message directly to the
host as an RSCP `distance` message, so publish feedback at a rate that is
meaningful to the judges — on every sensor reading, not every loop tick.

```python
#!/usr/bin/env python3
"""
exploration_node.py
-------------------
Action server: explore (rscp_mission_interfaces/Explore)

Goal    : (empty) — no parameters needed
Feedback: distance  — measurement in metres, forwarded directly to the host
Result  : success

Replace the placeholder logic with your real sensor reads and driving logic.
"""

import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from rscp_mission_interfaces.action import Explore


class ExplorationNode(Node):
    def __init__(self):
        super().__init__("exploration_node")

        self._server = ActionServer(
            self,
            Explore,
            "explore",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info("Exploration action server ready on 'explore'.")

    def _goal_callback(self, goal_request):
        self.get_logger().info("Exploration goal received. Starting lava tube traversal.")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("Exploration cancel requested.")
        return CancelResponse.ACCEPT

    async def _execute(self, goal_handle):
        self.get_logger().info("Beginning lava tube exploration...")

        feedback = Explore.Feedback()
        exploration_complete = False

        while not exploration_complete:
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Exploration cancelled.")
                goal_handle.canceled()
                return Explore.Result()

            # ------------------------------------------------------------------
            # YOUR CODE: drive forward, read distance sensor, check for
            # end-of-tube condition.
            #
            # Example structure:
            #   distance = self._read_distance_sensor()   # e.g. ultrasonic/lidar
            #   self._drive_forward(speed=0.2)
            #   if self._tube_exit_detected():
            #       exploration_complete = True
            # ------------------------------------------------------------------
            distance = 1.5          # replace with real sensor reading in metres
            exploration_complete = False  # replace with real exit condition

            # Publish the distance measurement as feedback.
            # The orchestrator forwards this directly to the host as an
            # RSCP distance message — publish only meaningful readings.
            feedback.distance = distance
            goal_handle.publish_feedback(feedback)
            self.get_logger().debug(f"Distance measurement: {distance} m")

            await asyncio.sleep(0.5)    # replace with your sensor read rate

        result = Explore.Result()
        result.success = True
        goal_handle.succeed()
        self.get_logger().info("Lava tube exploration complete.")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

**Test it without the orchestrator:**
```bash
ros2 action send_goal /explore rscp_mission_interfaces/action/Explore "{}"
```

---

## Where to put worker scripts

### Option A — Inside this package (simple workers)

For simple workers that don't need their own package, add the script directly
to this package's inner module folder:

```
rscp_mission_orchestrator/
├── rscp_mission_orchestrator/
│   ├── __init__.py
│   ├── mission_orchestrator.py
│   └── arm_control_node.py      <- add here
```

Then register it in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'mission_orchestrator = rscp_mission_orchestrator.mission_orchestrator:main',
        'arm_control_node = rscp_mission_orchestrator.arm_control_node:main',
    ],
},
```

Rebuild:

```bash
colcon build --packages-select rscp_mission_orchestrator
source install/setup.bash
```

### Option B — In their own package (recommended for team development)

When a worker is owned by a different teammate, it should live in its own
ROS2 package so it can be built, tested, and version-controlled independently:

```
src/
├── rscp_mission_orchestrator/
├── arm_control/                 # teammate's package
│   ├── arm_control/
│   │   ├── __init__.py
│   │   └── arm_control_node.py
│   ├── package.xml
│   └── setup.py
```

The teammate's `package.xml` must declare:

```xml
<depend>rclpy</depend>
<depend>std_srvs</depend>
<depend>rscp_mission_interfaces</depend>
```

### Adding any worker to the launch file

Once a worker is built and tested, add it to
`launch/rover_bringup.launch.py`. The commented placeholder entries are
already there — just uncomment and update the package and executable names:

```python
def generate_launch_description():
    # ... existing args and nodes ...

    arm_node = Node(
        package="arm_control",           # the worker's ROS2 package name
        executable="arm_control_node",   # the entry point name in setup.py
        name="arm_control_node",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        ...,        # existing entries
        arm_node,   # add here
    ])
```

Then rebuild and relaunch — the orchestrator connects automatically.

---

## Adding custom actions and services

You are not limited to the four built-in workers. You can add any custom
action or service to the stack — for example LED control, a camera trigger,
a gripper, a buzzer, or any other subsystem your rover has.

### Step 1 — Choose the right mechanism

| Your task | Use |
|---|---|
| Instant, on/off, pass/fail (LEDs, gripper, buzzer) | **Service** |
| Takes seconds/minutes, needs progress or cancellation | **Action** |
| Continuous stream of data with no request (IMU, GPS) | **Topic** (no worker needed — just publish) |

### Step 2 — Add the interface definition

All custom types go into `rscp_mission_interfaces` — the shared package
everyone already depends on. Never define a type inside your own worker
package.

**For a service**, create a `.srv` file in `rscp_mission_interfaces/srv/`:

```
rscp_mission_interfaces/
├── action/
│   ├── SearchArea.action
│   ├── NavigateToGPS.action
│   └── Explore.action
└── srv/
    └── SetLED.srv          <- example
```

**`SetLED.srv`** (fields before `---` are the request, after are the response):
```
bool enabled
uint8 r
uint8 g
uint8 b
---
bool success
string message
```

**For an action**, create a `.action` file in `rscp_mission_interfaces/action/`
(fields are goal `---` result `---` feedback):

```
rscp_mission_interfaces/
└── action/
    └── RunLightPattern.action   <- example
```

**`RunLightPattern.action`**:
```
# Goal
string pattern      # e.g. "blink", "pulse", "rainbow"
float32 duration    # seconds to run the pattern
---
# Result
bool success
---
# Feedback
float32 elapsed     # seconds elapsed so far
```

### Step 3 — Register in `CMakeLists.txt`

In `rscp_mission_interfaces/CMakeLists.txt`, add every new file to the
`rosidl_generate_interfaces` call:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SearchArea.action"
  "action/NavigateToGPS.action"
  "action/Explore.action"
  "action/RunLightPattern.action"   # new action
  "srv/SetLED.srv"                  # new service
)
```

Rebuild the interfaces package **before** building any worker that uses them:

```bash
colcon build --packages-select rscp_mission_interfaces
source install/setup.bash
```

Verify your new types exist:

```bash
ros2 interface list | grep rscp_mission_interfaces
# rscp_mission_interfaces/action/RunLightPattern
# rscp_mission_interfaces/srv/SetLED
# ... existing types
```

### Step 4 — Write the worker node

**Service worker example (LED on/off):**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rscp_mission_interfaces.srv import SetLED


class LEDControlNode(Node):
    def __init__(self):
        super().__init__("led_control_node")
        self._srv = self.create_service(
            SetLED, "set_led", self._handle
        )
        self.get_logger().info("LED service ready on 'set_led'.")

    def _handle(self, request, response):
        if request.enabled:
            self.get_logger().info(
                f"LED on: RGB=({request.r}, {request.g}, {request.b})"
            )
            # YOUR CODE: write to GPIO / serial / LED driver here
        else:
            self.get_logger().info("LED off.")
            # YOUR CODE: turn off hardware here

        response.success = True
        response.message = "OK"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LEDControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

**Action worker example (timed light pattern):**

```python
#!/usr/bin/env python3
import asyncio
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rscp_mission_interfaces.action import RunLightPattern


class LightPatternNode(Node):
    def __init__(self):
        super().__init__("light_pattern_node")
        self._server = ActionServer(
            self,
            RunLightPattern,
            "run_light_pattern",
            execute_callback=self._execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal: CancelResponse.ACCEPT,
        )
        self.get_logger().info("Light pattern action server ready.")

    async def _execute(self, goal_handle):
        pattern  = goal_handle.request.pattern
        duration = goal_handle.request.duration
        self.get_logger().info(f"Running pattern '{pattern}' for {duration}s")

        feedback = RunLightPattern.Feedback()
        elapsed = 0.0

        while elapsed < duration:
            if goal_handle.is_cancel_requested:
                # YOUR CODE: stop the pattern hardware here
                goal_handle.canceled()
                return RunLightPattern.Result()

            # YOUR CODE: drive the LED pattern hardware here

            await asyncio.sleep(0.1)
            elapsed += 0.1
            feedback.elapsed = elapsed
            goal_handle.publish_feedback(feedback)

        # YOUR CODE: stop the pattern hardware here
        result = RunLightPattern.Result()
        result.success = True
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = LightPatternNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### Step 5 — Call it from any node that needs it

A custom service or action is completely independent of the orchestrator —
any node in the stack can call it. For example, the search area worker could
turn the LED green when it finds the target:

```python
from rscp_mission_interfaces.srv import SetLED

class SearchAreaNode(Node):
    def __init__(self):
        super().__init__("search_area_node")
        # ... existing setup ...
        self._led = self.create_client(SetLED, "set_led")

    async def _execute(self, goal_handle):
        # ... search logic ...

        # Signal success with green LED
        req = SetLED.Request()
        req.enabled = True
        req.r, req.g, req.b = 0, 255, 0
        self._led.call_async(req)

        goal_handle.succeed()
        return SearchArea.Result()
```

Or add a client directly in `mission_orchestrator.py` and call it inside
`_handle_set_stage()` to change LED colour on every stage transition.

### Step 6 — Test it from the CLI

```bash
# Test a service
ros2 service call /set_led rscp_mission_interfaces/srv/SetLED \
  "{enabled: true, r: 0, g: 255, b: 0}"

# Test an action
ros2 action send_goal /run_light_pattern \
  rscp_mission_interfaces/action/RunLightPattern \
  "{pattern: 'blink', duration: 5.0}"
```

### Step 7 — Add to the launch file and rebuild

Register the new worker in `setup.py`, add it to `rover_bringup.launch.py`,
and rebuild:

```bash
colcon build --packages-select rscp_mission_interfaces rscp_mission_orchestrator
source install/setup.bash
```

---

## Common mistakes

| Mistake | Symptom | Fix |
|---|---|---|
| Wrong server name | `action server unavailable` in orchestrator logs | Match names exactly: `arm_disarm`, `search_area`, `navigate_to_gps`, `explore` |
| Using `def` instead of `async def` for `_execute` | Feedback never publishes; cancellation doesn't work; node freezes | Change to `async def _execute` and use `await asyncio.sleep()` in loops |
| Not calling `goal_handle.succeed()` before `return` | Orchestrator never sends `task_completed` to host | Always call `goal_handle.succeed()`, `goal_handle.abort()`, or `goal_handle.canceled()` before returning |
| Defining own `.action` file instead of using `rscp_mission_interfaces` | Action type mismatch — goal silently never connects | Remove custom definition; add `<depend>rscp_mission_interfaces</depend>` and import from there |
| Setting `response.success = True` even on failure | Orchestrator acks a failed arm — host thinks rover is armed | Only set `True` when the hardware operation actually succeeded |
| Not checking `goal_handle.is_cancel_requested` | Rover keeps driving after a new `set_stage` cancels the task | Add a check at the top of every loop in `_execute` |
| Building worker before interfaces | `ModuleNotFoundError` on `rscp_mission_interfaces` | Always build `rscp_mission_interfaces` first |

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `action server unavailable` for all actions | No worker nodes running yet — expected until workers are built |
| `action server unavailable` for one specific action | That worker is not running, wrong server name, or not yet built |
| Orchestrator sends `acknowledge` but no `task_completed` follows | Worker's `_execute` never calls `goal_handle.succeed()` — check worker logs |
| Goal rejected immediately | Worker's `_execute` crashes on the first call — check worker terminal for tracebacks |
| `arm_disarm` service times out | Arm worker not running or wrong service name |
| Type mismatch on action connection | Worker built against a different version of the `.action` file — rebuild `rscp_mission_interfaces` and the worker |
| Yellow import warning in VS Code | `rscp_mission_interfaces` not yet built or workspace not sourced — run `colcon build` then `source install/setup.bash` and reload VS Code |
