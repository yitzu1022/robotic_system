# Robotit Project Workspace Description

<a id="table-of-contents"></a>

## Table of Contents

- [1. Executive Summary](#section-1-executive-summary)
- [2. Repository Structure](#section-2-repository-structure)
- [3. Environment and Dependency Configuration](#section-3-environment-and-dependency-configuration)
- [4. Package Descriptions](#section-4-package-by-package-technical-analysis)
- [4.1 `decision_maker`](#section-41-decision_maker)
- [4.2 `mm_interface`](#section-42-mm_interface)
- [4.3 `get_pose`](#section-43-get_pose)
- [4.4 `kachaka_interfaces`](#section-44-kachaka_interfaces)
- [4.5 `kachaka_laser_api`](#section-45-kachaka_laser_api)
- [4.6 `kachaka_nav`](#section-46-kachaka_nav)
- [4.7 `map_alignment`](#section-47-map_alignment)
- [4.8 `object_query`](#section-48-object_query)
- [4.9 `object_query_interfaces`](#section-49-object_query_interfaces)
- [5. Custom ROS Interfaces Summary](#section-5-custom-ros-interfaces-summary)
- [6. Data Assets and Utility Scripts](#section-6-data-assets-and-utility-scripts)
- [6.1 `data/Util`](#section-61-datautil)
- [6.2 `data/lab`](#section-62-datalab)
- [7. Runtime Interaction Between Packages](#section-7-runtime-interaction-between-packages)
- [8. Practical Setup and Execution Guide](#section-8-practical-setup-and-execution-guide)

![Diagram](https://github.com/jimmy94828/robotic_system/blob/main/architecture.png)
<a id="section-1-executive-summary"></a>

## 1. Executive Summary

This repository is a ROS 2 workspace that integrates command interpretation, semantic object lookup, mobile navigation, map alignment, pose extraction, LiDAR acquisition, and a prototype semantic SLAM service. The workspace is organized around Python-based ROS 2 application packages and several interface packages that define custom actions and services. In addition to runtime code, the repository includes precomputed semantic map data, alignment assets, generated build outputs, and utility scripts for converting and validating map representations.

From a system perspective, the workspace implements a perception-to-action pipeline:

1. A user command is provided through text or audio.
2. The `decision_maker` package interprets the command into primitive robot actions.
3. The `object_query` package resolves semantic object names to coordinates from a semantic point-cloud map.
4. The `map_alignment` assets provide the transformation required to convert 3D semantic-map coordinates into the 2D navigation frame.
5. The `kachaka_nav` package plans and executes motion on either a real Kachaka robot or a simulation backend.
6. The `decision_maker` package optionally dispatches grasp/place actions through custom interfaces.
7. Supporting packages publish pose, LiDAR, and experimental semantic SLAM outputs.

The workspace therefore functions as a multi-package robotics application focused on semantic task execution in a mapped indoor environment.

<a id="section-2-repository-structure"></a>

## 2. Repository Structure

The top-level directory contains both source code and ROS 2 build artifacts:

```text
robot_ws/
├── src/                # ROS 2 source packages
├── data/               # Semantic maps, alignment files, utilities, and lab assets
├── build/              # Colcon build outputs
├── install/            # Installed package outputs
├── log/                # Historical colcon build logs
├── requirements.txt    # Python dependencies
└── environment.yml     # Conda environment descriptor
```

### 2.1 Source Packages

The `src/` directory contains the following ROS 2 packages:

| Package | Type | Purpose |
|---|---|---|
| `decision_maker` | `ament_python` | High-level task orchestration, command ingestion, and action dispatch |
| `mm_interface` | `ament_cmake` | Custom task, grasp, and place action definitions |
| `get_pose` | `ament_python` | TF-to-`PoseStamped` conversion utility |
| `kachaka_interfaces` | `ament_cmake` | Custom navigation action definition |
| `kachaka_laser_api` | `ament_python` | LiDAR acquisition from the Kachaka API and scan conversion |
| `kachaka_nav` | `ament_python` | Navigation servers, drivers, map utilities, calibration tools, and visualization |
| `map_alignment` | `ament_python` | Offline and online map alignment between 3D semantic maps and 2D LiDAR maps |
| `object_query` | `ament_python` | Semantic object lookup service over point-cloud map data |
| `object_query_interfaces` | `ament_cmake` | Object query service definition |
| `semantic_slam` | `ament_python` | Prototype semantic SLAM action server and client |
| `semantic_slam_interfaces` | `ament_cmake` | Semantic SLAM action definition |

### 2.2 Non-Source Directories

| Directory | Role |
|---|---|
| `build/` | Package-specific intermediate build outputs created by `colcon build` |
| `install/` | Generated runtime installation tree for ROS 2 packages |
| `log/` | Build and packaging logs from multiple historical `colcon` executions |
| `data/Util/` | Map conversion scripts, alignment YAML files, semantic metadata, and NPZ assets |
| `data/lab/` | Environment-specific lab maps, semantic point clouds, and map verification scripts |

The `build/`, `install/`, and `log/` directories are generated artifacts rather than hand-maintained source code. They are still relevant because they confirm that the workspace has been built repeatedly and recently.

<a id="section-3-environment-and-dependency-configuration"></a>

## 3. Environment and Dependency Configuration

### 3.1 `requirement.txt`

`requirement.txt` is a large environment rather than a minimal hand-curated dependency list. It includes:

- ROS 2 Python packages and ament tooling
- Scientific computing libraries such as `numpy`, `scipy`, and `matplotlib`
- Computer vision and visualization dependencies
- Speech-related packages
- Kachaka and Stretch ecosystem dependencies
- Machine learning and mapping support libraries

This file indicates that the workspace depends on a broader robotics and ML environment than what is declared in each package manifest individually.

### 3.2 `environment.yml`

`environment.yml` defines a minimal Conda environment named `robot_ros`. The file is sparse and does not enumerate the full set of dependencies required by the workspace. In practice, `requirements.txt` and the package manifests are more informative than `environment.yml`.

### 3.3 Docker Environment Setup

The repository root already contains:

- `/robotic-project/Dockerfile`
- `/robotic-project/entrypoint.sh`
- `/robotic-project/.dockerignore`

The Dockerfile:

- starts from `ros:humble-ros-base-jammy`
- installs ROS desktop tooling, RViz, rosbag, RealSense support, and build utilities
- builds `librealsense` from source
- installs Miniconda under `/opt/conda`
- creates the `robot_ros` Conda environment
- installs Python dependencies and Kachaka API support
- builds the ROS 2 workspace with `colcon`

The entrypoint script:

- sources `/opt/ros/humble/setup.bash`
- activates the `robot_ros` Conda environment
- rebuilds the workspace with `colcon build --symlink-install`
- sources `install/setup.bash`
- opens an interactive shell

#### 3.3.1 Build the Docker Image

```bash
cd /robotic
docker compose build
```
After the build completes, the image will be available locally as `robotic-project_robot_system:latest`.

Then run the container:

```bash
docker compose up -d
```

#### 3.3.2 Run the Container for Development
```bash
xhost +local:root

docker exec -it robot_system bash
```

<a id="section-4-package-by-package-technical-analysis"></a>

## 4. Package Descriptions

<a id="section-41-decision_maker"></a>

## 4.1 `decision_maker`

### Purpose

`decision_maker` is the orchestration layer that connects human command input, semantic lookup, coordinate conversion, navigation, and manipulation. In this repository it is the package that turns a short phrase such as `go to chair` or `bring bottle to table` into a concrete execution pipeline involving ROS topics, services, and actions.

### Main Files

| File | Role |
|---|---|
| `decision_maker/decision_maker_node.py` | Main execution node that plans command batches and dispatches navigation/manipulation requests |
| `decision_maker/decision_maker_test.py` | Experimental or test-oriented variant of the orchestration node |
| `decision_maker/text_command_node.py` | Reads terminal input and publishes text commands |
| `decision_maker/audio_command_node.py` | Records audio, runs speech-to-text, and publishes commands |
| `decision_maker/nl_command_node.py` | Terminal-based natural-language command frontend with live task-status feedback |
| `decision_maker/cancel_command_node.py` | Publishes cancellation requests |
| `decision_maker/mock_nav_server.py` | Mock navigation action server for testing |
| `decision_maker/mock_grasp_server.py` | Mock grasp/place action server for testing |
| `decision_maker/scenario_library.py` | Scenario templates that translate phrases into primitive execution steps |
| `decision_maker/nl_planner.py` | Lightweight world model used by scenarios to resolve objects and places |
| `decision_maker/command_types.py` | Command data abstraction |

### Core Behavior

At runtime, the package is organized as a three-stage chain:

1. A frontend node such as `nl_command_node.py` publishes a human sentence on `/manual_command`.
2. `decision_maker_node.py` matches the sentence against `SCENARIO_REGISTRY`, expands it into primitive steps, and stores the resulting batch in an internal queue.
3. The executor thread consumes those steps one by one and forwards them to `/object_query`, `/Navigate_to_pose`, and `/task_command`.

The package therefore separates:

- command intake
- symbolic scenario expansion
- world lookup
- frame conversion
- low-level action dispatch

That separation is important because it allows the planner logic in `scenario_library.py` and `nl_planner.py` to remain simple string-based code while `decision_maker_node.py` handles ROS integration, timeouts, visualization, and cancellation.

### `nl_command_node.py`

`nl_command_node.py` is the simplest user-facing entry point in the workspace. It implements a lightweight ROS 2 node named `nl_command_node` whose only job is to bridge terminal text input into the command pipeline.

Its implementation is intentionally minimal:

- it publishes `std_msgs/String` to `/manual_command`
- it subscribes to `/task_status`
- it creates a timer running every `0.1` seconds
- the timer uses `select.select()` on `sys.stdin` to perform non-blocking terminal polling

This design avoids a blocking `input()` call. Because stdin is polled inside a timer callback, the node can continue receiving and printing task feedback while the user is idle at the terminal.

Operationally, the file provides three behaviors:

- `send_command()` strips user text, publishes it, and logs the outgoing command
- `on_feedback()` classifies incoming status strings by prefix such as `done`, `failed`, and `cancel`
- `_poll_stdin()` checks whether a full terminal line is available and forwards it immediately

The node runs under a `MultiThreadedExecutor`, which is more than sufficient for this small workload and keeps the implementation consistent with the other multi-callback nodes in the repository. In the full system, this file is best understood as a console UI for the `decision_maker` package rather than as a planner or executor itself.

### `nl_planner.py`

`nl_planner.py` contains a small but important utility layer centered on the `WorldModel` class. It is not a full natural-language parser. Instead, it acts as the lookup and formatting helper used by scenario functions.

`WorldModel` is designed to be reused from an existing ROS node:

- if a node instance is passed into `WorldModel(node=self)`, it reuses that node's logger and ROS client context
- otherwise it creates a standalone node named `world_model_node`
- it always creates a client for `object_query_interfaces/ObjectQuery`

The file has two major responsibilities.

First, it resolves object names:

- `resolve_object(name)` normalizes the name to lowercase
- waits up to `3` seconds for `/object_query` to become available
- sends an asynchronous service request
- manually waits on the future with a timeout loop
- returns either `(x, y, z)` or `None`

This means all higher-level scenario code can treat semantic lookup as a normal Python function call.

Second, it resolves place names:

- `resolve_place(name)` first checks a hard-coded dictionary for symbolic places such as `me`, `home`, and `kitchen`
- if the place is not in the static table, it forwards the name to `resolve_object()`
- if that also fails, it falls back to `(0.0, 0.0, 0.0)` and logs a warning

That fallback is a notable design choice: scenario code calling `resolve_place()` will usually receive some coordinate tuple even for an unknown destination, which keeps execution moving but may also send the robot toward the origin if the label is unresolved.

The other key utility is `fmt_goto()`:

- it accepts either `(x, y)` or `(x, y, theta)`
- inserts a default heading of `0.0` when only two values are provided
- returns the normalized primitive string `goto:x,y,th`

This is the format expected by `decision_maker_node.py` during batch execution.

### `scenario_library.py`

`scenario_library.py` is the symbolic task-expansion layer. It does not talk to ROS directly. Instead, each function receives a `WorldModel` instance plus a command argument string and returns a `List[str]` of primitive actions.

Those primitives use a very small internal language:

- `goto:x,y,theta` for direct coordinate navigation
- `goto:name` for symbolic navigation that will be resolved later
- `grasp:item`
- `place:destination`

The main scenario functions are:

- `go_to_target(model, target)`: resolve a place or object and produce a single navigation step
- `give_item(model, item, dest="me")`: navigate to an object, grasp it, navigate to the destination, and place it
- `park_robot(model)`: return to `home`
- `clean_table(model)`: go to `table`, pick `trash`, and move it to `trash_bin` or `home`
- `fetch_drink(model)`: go from `fridge` to `me`
- `test_arm(model)`: emit a pure manipulation sequence for testing

The most implementation-heavy function is `go_from(model, arg)`, which parses commands of the form:

- `bring bottle to table`
- `bring the bottle on cabinet to table`
- `bring apple from shelf to me`

Its parsing strategy is deliberately lightweight:

- normalize to lowercase
- remove a leading `from`
- split source and destination on `to` or `into`
- strip articles such as `the`, `a`, and `an`
- detect prepositions such as `on`, `in`, `at`, `near`, `inside`, `under`, and `next to`
- if a prepositional phrase exists in the source, navigate to the location phrase rather than directly to the object label

For example, `bring bottle on cabinet to table` becomes a primitive sequence conceptually equivalent to:

- navigate to `cabinet`
- grasp `bottle`
- navigate to `table`
- place at `table`

The registry at the bottom of the file is the bridge into `decision_maker_node.py`:

- `SCENARIO_REGISTRY` maps fixed text prefixes such as `go to`, `bring`, `give me`, and `go home` to Python functions
- `decision_maker_node.py` performs prefix matching against this registry
- once a key matches, the remaining text is passed as that scenario's argument string

This file is therefore the policy layer for command meaning, while `decision_maker_node.py` is the runtime layer for execution.

### `decision_maker_node.py`

`decision_maker_node.py` is the package's central runtime component. It is the file that actually binds together command topics, scenario expansion, object-query lookup, map-frame conversion, navigation actions, manipulation actions, queueing, visualization, and cancellation.

The node starts by constructing several long-lived subsystems:

- `self.cmd_queue`: a bounded queue storing parsed command batches
- `self.world = WorldModel(node=self)`: shared world model for scenario functions
- `self.map2d_params`: optional 3D-to-2D calibration loaded from `map3d_to_map2d_yaml`
- `self.visualizer`: optional OpenCV map viewer loaded from `map_yaml`
- `self.nav_client`: `ActionClient` for `kachaka_interfaces/Navigate` on `/Navigate_to_pose`
- `self.task_client`: `ActionClient` for `TaskCommand` on `/task_command`
- `self.obj_client`: service client for `/object_query`

The main parameters are:

- `map3d_to_map2d_yaml`: alignment result used to project semantic 3D points into the 2D navigation frame
- `map_yaml`: occupancy-map style YAML used by the built-in `MapVisualizer`
- `grasp_approach_dist`: distance threshold that allows navigation to stop early when the next step is a grasp

The file also contains an embedded helper class, `MapVisualizer`, which:

- loads a map image and YAML metadata
- converts world coordinates to map pixels
- keeps a thread-safe image buffer
- runs a dedicated GUI thread for `cv2.imshow()` and `cv2.waitKey()`
- draws labels and markers for queried objects and navigation targets

The decision maker imports `TaskCommand` from `mm_interface.action` to send manipulation requests to grasp and place server. The interface is provided by the grasp and place part.

The command-handling flow is as follows.

1. `on_text_event()` receives a `String` from `/manual_command`.
2. The function scans `SCENARIO_REGISTRY` for the first matching prefix.
3. A background planning thread runs the corresponding scenario function so that blocking object-query calls do not stall the ROS executor.
4. `enqueue_command()` stores the resulting primitive batch in `self.cmd_queue`.
5. `command_executor_loop()` continuously dequeues batches and forwards them to `_execute_batch()`.

`_execute_batch()` is the primitive dispatcher. It iterates over action strings and sends them to:

- `_execute_nav()` for `goto:...`
- `_execute_grasp()` for `grasp:...`
- `_execute_place()` for `place:...`

The navigation path inside `_execute_nav()` is the most important part of the file.

It supports two forms of `goto`:

- direct coordinates such as `goto:-0.27,1.76,0.00`
- symbolic labels such as `goto:chair`

For coordinate form, it:

- parses the payload
- treats the incoming tuple as a semantic-map coordinate
- applies the loaded 3D-to-2D transform
- optionally draws the transformed target on the OpenCV map
- sends the converted goal to the `Navigate` action server

For symbolic form, it:

- calls `_query_object_position()`
- waits for the semantic service response
- applies the same transform inside that helper
- returns a navigation-frame coordinate tuple

During action execution, `_execute_nav()` also monitors `distance_remaining` feedback from the navigation action. If the next primitive is a grasp and the robot gets within `grasp_approach_dist`, the node cancels the remaining navigation path early and immediately proceeds to the grasp step. That shortcut is what allows the system to stop close enough for manipulation instead of insisting on exact pose convergence.

Manipulation is intentionally simpler but the latest code now behaves more like a grasp-plus-handover dispatcher than a generic place executor:

- `_execute_grasp()` waits `5` seconds, then converts `grasp:item` into a `TaskCommand` goal like `grasp the apple`
- `_execute_place()` converts `place:dest` into a handover-style command string: `handover <dest>` since the grasp and place server currently only supports handover behavior
- `_send_task_command()` handles goal sending, waiting, feedback logging, timeout, and success checking for both operations

The file also contains explicit cancellation and cleanup behavior:

- `/cancel_command` is listened to by `on_cancel_event()`
- `_send_cancel()` publishes `cancel` on `/task_status`
- `destroy_node()` shuts down worker threads and stops the OpenCV visualizer cleanly

At the bottom of the file, `load_map3d_to_map2d()` and `map3d_point_to_map2d_xy()` implement the actual projection math. They load the `plane_fit` and `sim2` blocks from the alignment YAML, project a 3D semantic point onto the fitted plane basis, then apply a 2D similarity transform. This is the exact bridge between the perception map used by `object_query` and the 2D map used by `kachaka_nav`.

<a id="section-42-mm_interface"></a>

## 4.2 `mm_interface`

### Purpose

`mm_interface` is a ROS 2 interface-only package. Its role is to define the action contract that the decision-making layer can use when sending textual manipulation or task commands to another server.

### Interface Files

| File | Description |
|---|---|
| `action/TaskCommand.action` | Generic text task action |

### `TaskCommand.action`

The action definition is intentionally minimal:

- Goal: `string command`
- Result: `bool success`, `string message`
- Feedback: `string feedback`

This design makes the action suitable as a generic bridge between symbolic task planning and an external manipulation or behavior-execution module. The planner side does not need to know whether the downstream server is controlling a gripper, an arm, a handover behavior, or a scripted skill. It only needs to send a textual instruction and wait for structured success or failure.

<a id="section-43-get_pose"></a>

## 4.3 `get_pose`

### Purpose

`get_pose` is a small utility package that republishes a TF transform as a `PoseStamped` topic.

### Main File

| File | Role |
|---|---|
| `get_pose/tf_to_pose.py` | Looks up a TF transform and publishes it as `pose` |

### Behavior

`TfToPose`:

- reads `target_frame`, `base_frame`, and `rate_hz` parameters
- listens to TF using `Buffer` and `TransformListener`
- looks up the transform from `target_frame` to `base_frame`
- republishes the result on the `pose` topic as `geometry_msgs/PoseStamped`

This is a practical bridge utility for logging, debugging, rosbag processing, or downstream tooling that prefers pose messages over TF lookups.

<a id="section-44-kachaka_interfaces"></a>

## 4.4 `kachaka_interfaces`

### Purpose

This package defines the navigation action contract used throughout the workspace.

### Interface File

| File | Description |
|---|---|
| `action/Navigate.action` | Navigation goal with target coordinates, result status, and distance feedback |

The `Navigate` action is the primary motion interface used by `decision_maker` and implemented by navigation servers in `kachaka_nav`.

<a id="section-45-kachaka_laser_api"></a>

## 4.5 `kachaka_laser_api`

### Purpose

This package connects to the Kachaka robot API and republishes robot LiDAR data into ROS 2.

### Main Files

| File | Role |
|---|---|
| `kachaka_laser_api/kachaka_laser_from_api_node.py` | Retrieves LiDAR scans from the Kachaka API and publishes `LaserScan` |
| `kachaka_laser_api/scan2ptcloud.py` | Converts laser scans into point-cloud representations |

### Core Behavior

`kachaka_laser_from_api_node.py`:

- connects to a real Kachaka robot through `kachaka_api.KachakaApiClient`
- publishes `sensor_msgs/LaserScan`
- supports configurable topic name, frame override, publish rate, cursor-based deduplication, and reconnection policy
- runs with callback groups and a multi-threaded executor to separate timer and I/O behavior

<a id="section-46-kachaka_nav"></a>

## 4.6 `kachaka_nav`

### Purpose

`kachaka_nav` is the motion-execution package. It exposes the custom `Navigate` action, translates user-frame goals into the robot's native frame, talks to either the real Kachaka hardware or a simulator driver, and continuously republishes the robot pose and path for the rest of the system.

### Main Runtime Files

| File | Role |
|---|---|
| `kachaka_nav/modular_nav.py` | Main lightweight navigation action server used by the integrated pipeline |
| `kachaka_nav/robot_driver.py` | Driver abstraction for real Kachaka hardware and ROS simulation |

### Navigation Architecture

The package is built around two layers:

1. `Navigate.action` as the control contract seen by upstream packages.
2. `robot_driver.py` as the concrete backend for either real hardware or simulation.

`robot_driver.py` provides two interchangeable implementations:

- `KachakaRealDriver` for direct communication with the physical Kachaka platform
- `RosSimDriver` for simulation-oriented control through ROS topics

`modular_nav.py` is the runtime bridge between those drivers and the rest of the workspace.

### `modular_nav.py`

`modular_nav.py` implements `ModularNavNode`, a ROS 2 node that acts as a navigation action server and a pose-republishing bridge at the same time.

Its startup logic is structured around four concerns.

First, it declares configuration parameters:

- `use_sim`: select the simulator driver or the real robot driver
- `kachaka_ip`: network endpoint for the real robot
- `user_map_yaml`: optional YAML file whose `origin` field defines the offset and yaw between the user map and the Kachaka-native map
- `goal_xy_tolerance`: planar tolerance used to decide when a goal is considered reached, defaulting to `0.6` meters in the latest file

Second, it configures callback concurrency:

- a `MutuallyExclusiveCallbackGroup` is used for timer-driven pose publishing
- a `ReentrantCallbackGroup` is used for the `Navigate` action server
- the node runs under a `MultiThreadedExecutor`

This prevents the pose-publication loop and the action-execution loop from blocking each other.

Third, it creates the runtime publishers and TF output:

- `/user_pose` publishes the pose transformed into the user-defined map frame
- `/kachaka_pose` publishes the robot pose in the Kachaka-native `map` frame
- `/robot_path` appends the motion trajectory as a `nav_msgs/Path`
- a TF transform from `map` to `base_link` is broadcast continuously

Fourth, it selects the navigation backend:

- `RosSimDriver(self)` when `use_sim` is true
- `KachakaRealDriver(robot_ip)` otherwise

The frame-conversion logic is explicit and local to this file:

- `load_map_alignment()` reads the `origin` field from a YAML file
- `transform_user_to_kachaka()` rotates and translates a goal from the user map into the robot's native map
- `transform_kachaka_to_user()` performs the inverse conversion for published pose output

The periodic pose path in `publish_pose_callback()` does the following every `0.1` seconds:

- queries the current pose from the active driver
- converts yaw into a quaternion
- publishes a `PoseStamped` in the native `map` frame
- broadcasts the matching TF transform
- appends that pose to the path history, capped at `8000` samples
- converts the same pose into `user_map` coordinates and publishes it on `/user_pose`

The action server is implemented in `execute_callback()`. When a `Navigate` goal arrives:

- it interprets the request as a goal in the user frame
- converts it into native Kachaka coordinates
- calls `driver.move_native()` in non-blocking mode when possible
- repeatedly polls the robot pose
- computes Euclidean distance to the goal
- publishes `distance_remaining` feedback
- stops navigation successfully once the goal is within `goal_xy_tolerance`

The callback also contains explicit timeout and cancellation behavior:

- if execution exceeds `60` seconds, it tries `cancel_current_command()` and `stop()`
- when the goal is close enough, it also cancels/stops the robot's internal motion to avoid overshooting
- it returns a `Navigate.Result(success=...)` and marks the goal as succeeded or aborted accordingly

This node assumes upstream code already solved the semantic-navigation problem. `modular_nav.py` does not perform object lookup, semantic reasoning, or 3D-to-2D projection itself. It expects `decision_maker_node.py` to hand it a final 2D goal in the user map frame, then focuses purely on:

- frame conversion between user map and Kachaka map
- motion execution through the selected driver
- progress feedback through `distance_remaining`
- pose and path publication for monitoring

In the integrated system, this file is therefore the execution endpoint that receives 2D targets from `decision_maker_node.py` after semantic lookup and map alignment have already been completed.

<a id="section-47-map_alignment"></a>

## 4.7 `map_alignment`

### Purpose

`map_alignment` solves the calibration problem between a 3D semantic map and a 2D LiDAR or occupancy map. This is a critical subsystem because semantic object lookup is performed in a 3D reconstructed map, while robot navigation operates in a 2D motion-planning frame.

### Main Files

| File | Role |
|---|---|
| `map_alignment/map_alignment_v2.py` | Main offline-friendly alignment pipeline |
| `map_alignment/collect_data.py` | Data capture utility for synchronized sensor and pose collection |
| `map_alignment/lidar_camera_link_bridge.py` | TF bridge between lidar and camera frames |

### Core Capabilities

`map_alignment_v2.py` performs:

- loading of offline 2D and 3D pose sequences
- parsing of quaternion conventions and pose conventions
- optional camera-to-base transformation via extrinsic calibration
- timestamp normalization, scale compensation, and time-offset search
- nearest-neighbor trajectory synchronization
- RANSAC-based SE(2) estimation
- nonlinear refinement of translation, yaw, and optional scale
- estimation of z-offset between map frames
- generation of a static TF transform from `map_2d` to `map_3d`
- export of YAML results and a ready-to-run `static_transform_publisher` command script

This package is central to the semantic navigation workflow because the `decision_maker` package loads the alignment result from `data/Util/alignment.yaml` to transform semantic 3D object coordinates into 2D navigation targets.

### `collect_data.py`

This script is a substantial data-acquisition utility. It includes:

- TF pose capture
- RealSense image/depth handling
- file organization for output datasets
- synchronized sample export

It supports calibration dataset generation rather than online navigation directly.

<a id="section-48-object_query"></a>

## 4.8 `object_query`

### Purpose

`object_query` is the semantic-memory package of the workspace. It loads offline semantic-map assets, organizes them into a searchable in-memory database, serves object-location queries through ROS 2, and publishes visualization outputs that make those semantic results visible in RViz.

### Main Files

| File | Role |
|---|---|
| `object_query/object_query_server.py` | Main semantic object query server and map-visualization publisher |
| `object_query/object_query_client.py` | Simple service client |
| `object_query/object_query_test.py` | Expanded test or experimental server variant |

### Core Behavior

The package's central implementation is `object_query_server.py`.

### `object_query_server.py`

`object_query_server.py` defines `ObjectQueryServer`, a ROS 2 node that combines three responsibilities:

- loading semantic and geometric map data from disk
- answering semantic lookup requests through `/object_query`
- publishing RViz-friendly point clouds and markers

The file begins by declaring several map-related parameters:

- `3dmap_path`: dense 3D scene point cloud
- `map_path`: semantic NPZ map containing points and semantic IDs
- `semantic_path`: JSON metadata describing segment IDs and class names
- `instance_path`: optional instance-level JSON with centroids per object instance
- `auto_align`: optional flag to rotate the semantic map so floor-like classes align with the XY plane

At startup the node creates:

- a service server on `/object_query`
- a publisher on `/object_list`
- a `MarkerArray` publisher on `/semantic_map_markers`
- a `PointCloud2` publisher on `/map_pointcloud`

It also initializes an in-memory database:

- `self.object_db` is a `defaultdict(list)`
- each key is a lowercased semantic label such as `chair` or `bottle`
- each value is a list of 3D centroids

The loading path has two modes.

1. Preferred path: `load_instance_map()`

If `instance_path` exists, the server reads instance-level annotations from JSON:

- it expects an `instances` dictionary
- each instance contributes `semantic_name` and `centroid`
- centroids are grouped by semantic class name into `object_db`
- the dense 3D map is loaded separately for visualization

This mode gives the system direct object-instance centroids instead of deriving them from broad semantic segments.

2. Fallback path: `load_semantic_map()`

If no instance-level file exists, the server reconstructs the database from the semantic map itself:

- loads semantic points from either `pts` or `means3D`
- loads semantic IDs from either `pan` or `semantic_ids`
- loads the separate dense 3D map and optional colors
- parses semantic metadata from JSON fields such as `segments_info`, `segmentation`, or `segments`
- builds an `id_to_name` mapping from segment ID to category name
- optionally computes a global alignment rotation using floor-like classes
- builds one centroid per semantic segment by averaging its bounding-box minimum and maximum corners

The alignment routine in `compute_alignment_matrix()` is also worth noting:

- it searches for floor-related labels such as `floor`, `ground`, `carpet`, `tile`, and `wood`
- extracts the corresponding points
- fits a dominant plane using SVD
- computes the rotation needed to align the estimated floor normal with the Z axis
- applies the same rotation to both the semantic map and the dense 3D point cloud

Once the data is loaded, the server immediately:

- publishes the serialized object database on `/object_list`
- publishes the dense 3D map as `PointCloud2`
- starts a timer that republishes the point cloud every second

The query path is implemented by `handle_query()`:

- normalize the request name to lowercase
- call `search_object()`
- return `found`, `position`, and a human-readable message
- if successful, clear old markers and publish markers for the requested class only
- if unsuccessful, clear all markers

The visualization behavior is intentionally query-scoped. Rather than publishing the full semantic map as labels all the time, `publish_object_marker()` displays only the currently queried object class:

- a green sphere marker for each instance
- a text marker above each sphere
- `Marker.DELETEALL` before every new query so the view stays uncluttered

`publish_point_cloud()` builds a raw `sensor_msgs/PointCloud2` message manually:

- uses the loaded 3D map points
- packs RGB values into a float field
- publishes the data in the `map` frame

From the perspective of the overall system, this node is the source of truth for semantic object positions. `decision_maker/nl_planner.py` and `decision_maker/decision_maker_node.py` both rely on this service to convert a label like `chair` into a concrete 3D location before navigation and manipulation can proceed.

<a id="section-49-object_query_interfaces"></a>

## 4.9 `object_query_interfaces`

### Purpose

This package defines the semantic lookup service contract.

### Interface File

| File | Description |
|---|---|
| `srv/ObjectQuery.srv` | Request object name, return found flag, 3D position, and message |

This service is the main interface between symbolic task commands and semantic map perception.

<a id="section-5-custom-ros-interfaces-summary"></a>

## 5. Custom ROS Interfaces Summary

The workspace defines several custom interfaces that form the contract between packages:

| Package | Interface | Function |
|---|---|---|
| `kachaka_interfaces` | `Navigate.action` | Motion request to a target coordinate |
| `object_query_interfaces` | `ObjectQuery.srv` | Semantic lookup by object name |
| `mm_interface` | `TaskCommand.action` | Generic task submission |

<a id="section-6-data-assets-and-utility-scripts"></a>

## 6. Data Assets and Utility Scripts

<a id="section-61-datautil"></a>

## 6.1 `data/Util` Folder

The `data/Util/` directory contains conversion tools and baseline map assets used by semantic querying and alignment.

### Important Files

| File | Role |
|---|---|
| `Final_GS.npz` | 3D point-cloud map asset |
| `Final_SEM_GS_converted.npz` | Semantic map asset used by object-query nodes |
| `Final_SEM_GS_converted_meta.json` | Metadata describing semantic segments/categories |
| `alignment.yaml` | Main 3D-to-2D alignment file consumed by `decision_maker` |
| `semantic_legend.json` | Semantic category/color reference |
| `actual_color_mapping.json` | Color mapping asset |

### Utility Scripts

| File | Role |
|---|---|
| `ply_to_npz.py` | Converts PLY point clouds to NPZ |
| `read_ply.py` | Reads PLY point clouds |
| `read_npz.py` | Reads NPZ map files |
| `downsample.py` | Downsampling utility |
| `convert_sem_gs_exact.py` | Converts ply semantic Gaussian-splatting outputs to NPZ format |
| `sample_timestamp.py` | Sampling helper for timestamps |
| `get_timestamp.py` | Timestamp extraction helper |
| `renumber_pairs.py` | Renumbering helper for paired data |

These scripts indicate that the workspace includes a full preprocessing pipeline for converting external reconstruction or semantic segmentation outputs into ROS-consumable map artifacts.

<a id="section-62-datalab"></a>

## 6.2 `data/lab`

The `data/lab/` directory appears to be a concrete environment dataset for a lab setting.

### Important Files

| File | Role |
|---|---|
| `kachaka_map.pgm` and `kachaka_map.yaml` | Occupancy map representation |
| `kachaka_native.png` and `kachaka_native.yaml` | Native map exported from the Kachaka ecosystem |
| `accumulated_gaussians.npz` | 3D Gaussian map asset |
| `semantic_pcd_accumulated_gaussians.npz` | Semantic point-cloud map asset |
| `semantic_pcd_accumulated_gaussians_meta.json` | Semantic metadata for the lab map |
| `accumulated_gaussians_instance_semantic_info.json` | Instance-level semantic annotations |
| `pred_trajectory.json` | Trajectory data for validation or alignment |

### Utility Scripts

| File | Role |
|---|---|
| `align_map.py` | Map comparison/alignment tool |
| `auto_align.py` | Automatic alignment utility |
| `export_kachaka_map.py` | Kachaka map export helper |
| `check.py` | Validation or inspection helper |

This directory functions as a concrete example dataset and experimental workspace for map alignment and semantic-navigation development.

<a id="section-7-runtime-interaction-between-packages"></a>

## 7. Runtime Interaction Between Packages

The intended integrated workflow is as follows:

### 7.1 Command Intake

- `text_command_node` or `audio_command_node` publishes user intent to `/manual_command`.

### 7.2 Task Interpretation

- `decision_maker_node` receives the command.
- `scenario_library.py` converts the utterance into primitive navigation and manipulation actions.

### 7.3 Semantic Lookup

-  `DecisionMakingNode` calls `ObjectQuery.srv`.
- `object_query_server.py` returns a centroid in the semantic 3D map frame.

### 7.4 Frame Conversion

- `decision_maker_node.py` loads `data/Util/alignment.yaml`.
- the returned 3D object location is projected and transformed into the 2D navigation frame.

### 7.5 Navigation Execution

- `decision_maker_node.py` sends a `Navigate` action goal.
- `kachaka_nav` receives the action and plans a path.
- `robot_driver.py` dispatches low-level velocity or robot-native commands.

### 7.6 Manipulation Execution

- `decision_maker_node.py` sends `TaskCommand` to the robot arm manipulation node.
- mock servers can emulate this during testing.

### 7.7 Visualization and Debugging

- `object_query` publishes queried markers and point clouds.
- `decision_maker` can show target markers on a 2D map via OpenCV.
- `get_pose` can republish TF as `PoseStamped`.
- `kachaka_laser_api` publishes `LaserScan` for sensing and debugging.

This interaction pattern confirms that the workspace is not a collection of isolated experiments; it is an integrated semantic-task robotics stack centered on ROS 2 message passing and action/service composition.

<a id="section-8-practical-setup-and-execution-guide"></a>

## 8. Setup and Execution Guide

This section adds an operator-oriented guide for building the environment and launching the key packages discussed above. The commands below are based on the current repository layout under `/robotic-project`.

### 8.1 Path Assumptions

The instructions in this section assume:

- project root: `/robotic_system`
- ROS 2 workspace root: `/robotic_system/robot_ws`
- ROS distribution: `humble`
- Conda environment name: `robot_ros`

Because several packages use relative default paths such as `data/Util/alignment.yaml` and `data/lab/kachaka_native.yaml`, it is safer to either:

- run commands from the workspace root, or
- pass absolute paths through `--ros-args -p ...`

The examples below use absolute paths to avoid ambiguity.

### 8.2 Local Conda Environment Setup

The repository already includes:

- `/robotic-project/robot_ws/environment.yml`
- `/robotic-project/robot_ws/requirements.txt`
- `/robotic-project/Dockerfile`

The Dockerfile is the most complete reference for the intended Python environment. A local Conda workflow equivalent to that Dockerfile is shown below.

#### 8.2.1 Create and Activate the Environment

```bash
cd /robotic-project/robot_ws

source "$(conda info --base)/etc/profile.d/conda.sh"
conda env create -f environment.yml || conda create -n robot_ros python=3.8 -y
conda activate robot_ros
```

#### 8.2.2 Install Python Tooling Used by the Workspace

```bash
pip install --upgrade pip setuptools wheel
pip uninstall -y em || true
pip install Cython
pip install catkin_pkg lxml
pip install empy==3.3.4
pip install lark-parser
pip install kachaka-api==3.14.4.0
pip install pyyaml tqdm transforms3d
```

#### 8.2.3 Install Packages That the Dockerfile Installs Through Conda

The Dockerfile installs several packages through Conda rather than `pip`, mainly for compatibility on ARM and robotics systems:

```bash
conda install -n robot_ros -c conda-forge -y grpcio=1.59.3
conda install -n robot_ros -c conda-forge -y open3d
conda install -n robot_ros -c conda-forge -y pyrealsense2 || true
```

#### 8.2.4 Install the Workspace Python Requirements

```bash
pip install -r /robotic-project/robot_ws/requirements.txt
```

#### 8.2.5 Make ROS Python Packages Visible Inside the Conda Environment

The Dockerfile adds ROS Humble site-packages into the Conda environment through a `.pth` file. The same idea can be applied locally:

```bash
python - <<'PY'
import sysconfig
from pathlib import Path

purelib = Path(sysconfig.get_paths()["purelib"])
(purelib / "ros_humble_local.pth").write_text("/opt/ros/humble/lib/python3.8/site-packages\n")
PY
```

If the local ROS installation uses a different Python minor version, adjust the path accordingly.

#### 8.2.6 Build the Workspace

```bash
cd /robotic-project/robot_ws
source /opt/ros/humble/setup.bash
conda activate robot_ros
colcon build --merge-install --symlink-install
source install/setup.bash
```

#### 8.2.7 Recommended Shell Initialization for Every New Terminal

```bash
cd /robotic-project/robot_ws
source /opt/ros/humble/setup.bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate robot_ros
source install/setup.bash
export PYTHONPATH=$PYTHONPATH:/opt/conda/envs/robot_ros/lib/python3.10/site-packages
```

If the environment is local rather than container-based, replace `/opt/conda/envs/robot_ros/...` with the local Conda environment path shown by `conda info --envs`.



### 8.4 Workspace Build Commands

Whether using the local Conda setup or the container, the standard workspace build sequence is:

```bash
cd /robotic-project/robot_ws
source /opt/ros/humble/setup.bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate robot_ros
colcon build --merge-install --symlink-install
source install/setup.bash
```

If you are already inside the Docker container, replace the workspace path with `/robot_ws`.

### 8.5 Package Execution Commands

This subsection provides the practical `ros2 run` commands for the packages the repository uses most often in the semantic-task pipeline.

### 8.5.1 Start `object_query`

This node loads the semantic map, publishes the point cloud and queried markers, and exposes the `ObjectQuery` service.

```bash
cd /robotic-project/robot_ws
source /opt/ros/humble/setup.bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate robot_ros
source install/setup.bash

ros2 run object_query object_query_server
```

### 8.5.2 Start `kachaka_nav`

#### Real Robot Example (Kachaka moving platform)

```bash
cd /robotic-project/robot_ws
source /opt/ros/humble/setup.bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate robot_ros
source install/setup.bash

ros2 run kachaka_nav modular_nav_node
```

### 8.5.3 Start `decision_maker` node

The main executable is `decision_maker_node`. It expects:

- the `ObjectQuery` service to be available
- a navigation action server to be running
- a valid 3D-to-2D alignment YAML file
- optionally, a valid 2D map YAML for OpenCV visualization

```bash
cd /robotic-project/robot_ws
source /opt/ros/humble/setup.bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate robot_ros
source install/setup.bash

ros2 run decision_maker decision_maker_node --ros-args \
  -p map3d_to_map2d_yaml:=/robotic-project/robot_ws/data/Util/alignment.yaml \
  -p map_yaml:=/robotic-project/robot_ws/data/lab/kachaka_native.yaml
```

### 8.5.4 Start `nl_command_node`

`nl_command_node` is a terminal-based natural-language command publisher. It publishes text to `/manual_command` and displays feedback from `/task_status`.

```bash
cd /robotic-project/robot_ws
source /opt/ros/humble/setup.bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate robot_ros
source install/setup.bash

ros2 run decision_maker nl_command_node
```

Example input after the node starts:

```text
go to chair
bring bottle to table
bring apple on cabinet to table
give me apple
go home
```

### 8.6 Run the System in Multi-Terminal

For an integrated manual test of the semantic navigation stack, open several terminals and launch the nodes in the following.

#### Terminal 1: Object Query Service

```bash
cd /robotic-project/robot_ws
source /opt/ros/humble/setup.bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate robot_ros
source install/setup.bash
ros2 run object_query object_query_server --ros-args \
  -p 3dmap_path:=/robotic-project/robot_ws/data/Util/Final_GS.npz \
  -p map_path:=/robotic-project/robot_ws/data/Util/Final_SEM_GS_converted.npz \
  -p semantic_path:=/robotic-project/robot_ws/data/Util/Final_SEM_GS_converted_meta.json
```

#### Terminal 2: Navigation Server

```bash
cd /robotic-project/robot_ws
source /opt/ros/humble/setup.bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate robot_ros
source install/setup.bash
ros2 run kachaka_nav modular_nav_node --ros-args \
  -p use_sim:=false \
  -p kachaka_ip:=192.168.0.157:26400 \
  -p use_native_map:=true
```

#### Terminal 3: Decision Maker Node

```bash
cd /robotic-project/robot_ws
source /opt/ros/humble/setup.bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate robot_ros
source install/setup.bash
ros2 run decision_maker decision_maker_node --ros-args \
  -p map3d_to_map2d_yaml:=/robotic-project/robot_ws/data/Util/alignment.yaml \
  -p map_yaml:=/robotic-project/robot_ws/data/lab/kachaka_native.yaml
```

#### Terminal 4: Natural-Language Command Input

```bash
cd /robotic-project/robot_ws
source /opt/ros/humble/setup.bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate robot_ros
source install/setup.bash
ros2 run decision_maker nl_command_node
```

At this point, type commands such as `go to chair` or `bring bottle to table` in the `nl_command_node` terminal.
