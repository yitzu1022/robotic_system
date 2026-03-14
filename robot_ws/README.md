# Robot Workspace Technical Report

<a id="table-of-contents"></a>

## Table of Contents

- [1. Executive Summary](#section-1-executive-summary)
- [2. Repository Structure](#section-2-repository-structure)
- [3. Environment and Dependency Configuration](#section-3-environment-and-dependency-configuration)
- [4. Package-by-Package Technical Analysis](#section-4-package-by-package-technical-analysis)
- [4.1 `decision_maker`](#section-41-decision_maker)
- [4.2 `decision_maker_interfaces`](#section-42-decision_maker_interfaces)
- [4.3 `get_pose`](#section-43-get_pose)
- [4.4 `kachaka_interfaces`](#section-44-kachaka_interfaces)
- [4.5 `kachaka_laser_api`](#section-45-kachaka_laser_api)
- [4.6 `kachaka_nav`](#section-46-kachaka_nav)
- [4.7 `map_alignment`](#section-47-map_alignment)
- [4.8 `object_query`](#section-48-object_query)
- [4.9 `object_query_interfaces`](#section-49-object_query_interfaces)
- [4.10 `semantic_slam`](#section-410-semantic_slam)
- [4.11 `semantic_slam_interfaces`](#section-411-semantic_slam_interfaces)
- [5. Custom ROS Interfaces Summary](#section-5-custom-ros-interfaces-summary)
- [6. Data Assets and Utility Scripts](#section-6-data-assets-and-utility-scripts)
- [6.1 `data/Util`](#section-61-datautil)
- [6.2 `data/lab`](#section-62-datalab)
- [7. Runtime Interaction Between Packages](#section-7-runtime-interaction-between-packages)
- [8. Testing and Quality Support](#section-8-testing-and-quality-support)
- [9. Observations on Codebase Maturity](#section-9-observations-on-codebase-maturity)
- [10. Key Files Worth Immediate Attention](#section-10-key-files-worth-immediate-attention)
- [11. Conclusion](#section-11-conclusion)
- [12. Practical Setup and Execution Guide](#section-12-practical-setup-and-execution-guide)

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
├── requirements.txt    # Python dependency snapshot
└── environment.yml     # Minimal Conda environment descriptor
```

### 2.1 Source Packages

The `src/` directory contains the following ROS 2 packages:

| Package | Type | Purpose |
|---|---|---|
| `decision_maker` | `ament_python` | High-level task orchestration, command ingestion, and action dispatch |
| `decision_maker_interfaces` | `ament_cmake` | Custom task, grasp, and place action definitions |
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

### 3.1 `requirements.txt`

`requirements.txt` is a large environment snapshot rather than a minimal hand-curated dependency list. It includes:

- ROS 2 Python packages and ament tooling
- Scientific computing libraries such as `numpy`, `scipy`, and `matplotlib`
- Computer vision and visualization dependencies
- Speech-related packages
- Kachaka and Stretch ecosystem dependencies
- Machine learning and mapping support libraries

This file indicates that the workspace depends on a broader robotics and ML environment than what is declared in each package manifest individually.

### 3.2 `environment.yml`

`environment.yml` defines a minimal Conda environment named `robot_ros`. The file is sparse and does not enumerate the full set of dependencies required by the workspace. In practice, `requirements.txt` and the package manifests are more informative than `environment.yml`.

<a id="section-4-package-by-package-technical-analysis"></a>

## 4. Package-by-Package Technical Analysis

<a id="section-41-decision_maker"></a>

## 4.1 `decision_maker`

### Purpose

`decision_maker` is the central orchestration package. It receives human commands, transforms them into primitive actions, queries semantic object locations, converts those coordinates into the navigation frame, and dispatches navigation and manipulation requests to downstream servers.

### Main Files

| File | Role |
|---|---|
| `decision_maker/decision_maker_node.py` | Primary task-execution node for the integrated system |
| `decision_maker/decision_maker_node_calib.py` | Calibration-oriented variant of the main decision node |
| `decision_maker/decision_maker_node_isaacsim.py` | Variant adapted for Isaac Sim style integration |
| `decision_maker/decision_maker_test.py` | Experimental or test-oriented variant of the orchestration node |
| `decision_maker/text_command_node.py` | Reads terminal input and publishes text commands |
| `decision_maker/audio_command_node.py` | Records audio, runs speech-to-text, and publishes commands |
| `decision_maker/audio_command_node_p2t.py` | Alternate audio command implementation |
| `decision_maker/nl_command_node.py` | Natural-language command publisher node |
| `decision_maker/cancel_command_node.py` | Publishes cancellation requests |
| `decision_maker/mock_nav_server.py` | Mock navigation action server for testing |
| `decision_maker/mock_grasp_server.py` | Mock grasp/place action server for testing |
| `decision_maker/scenario_library.py` | Maps human phrases to multi-step task sequences |
| `decision_maker/nl_planner.py` | World model and command-to-primitive formatting utilities |
| `decision_maker/command_types.py` | Command data abstraction |
| `launch/decision_with_text.launch.py` | Launches the main decision node with a text command source |
| `launch/mock_actions.launch.py` | Launches mock navigation and grasp action servers |
| `launch/mock_actions_isaacsim.launch.py` | Launches mock grasp support for simulator-oriented workflows |
| `stretch/audio/*.py` | Audio support utilities for recording, speech-to-text, and text-to-speech |

### Core Behavior

The central implementation is `decision_maker_node.py`. Its responsibilities are:

- subscribe to `/manual_command` and `/cancel_command`
- maintain a bounded command queue
- use `scenario_library.py` to parse high-level tasks such as `go to`, `bring`, `give me`, `fetch drink`, and `clean table`
- resolve semantic objects and named places through `WorldModel`
- call the `ObjectQuery` service to retrieve object coordinates
- load a 3D-to-2D alignment YAML file and convert semantic-map coordinates into the 2D navigation frame
- optionally display queried targets on a 2D map through an OpenCV visualizer
- send navigation goals through the custom `Navigate` action
- send manipulation goals through the `GraspPlace` action
- publish task status and cancellation signals

### `scenario_library.py`

This file provides reusable scenario templates. Rather than hard-coding one command path, it converts human-readable intent into primitive strings such as:

- `goto:x,y,theta`
- `grasp:object_name`
- `place:target_name`

Examples include:

- `go_to_target`
- `give_item`
- `park_robot`
- `clean_table`
- `fetch_drink`
- `go_from`
- `test_arm`

This design separates command semantics from execution mechanics.

### `nl_planner.py`

`nl_planner.py` contains the `WorldModel` class. It bridges semantic queries and static place definitions:

- static places such as `me`, `home`, and `kitchen` are resolved locally
- unknown places are forwarded to the `ObjectQuery` service
- objects are always resolved through the semantic map service
- `fmt_goto()` formats pose tuples into the primitive `goto:` command syntax expected by the executor

This package is therefore the task-level coordination layer of the workspace.

<a id="section-42-decision_maker_interfaces"></a>

## 4.2 `decision_maker_interfaces`

### Purpose

This package defines custom ROS 2 actions used by the orchestration layer for manipulation and task dispatch.

### Interface Files

| File | Description |
|---|---|
| `action/TaskCommand.action` | Generic text task action |
| `action/Grasp.action` | Grasp-only action with success/message feedback |
| `action/Place.action` | Place-only action with success/message feedback |
| `action/GraspPlace.action` | Combined manipulation action carrying `object_id` and `target_bin` |

These actions provide the type contracts required by nodes in `decision_maker`.

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

This package is a hardware-facing integration layer that exposes Kachaka LiDAR data to the ROS graph.

<a id="section-46-kachaka_nav"></a>

## 4.6 `kachaka_nav`

### Purpose

`kachaka_nav` provides navigation action servers, path planning libraries, robot drivers, coordinate conversion helpers, and several map visualization and calibration utilities.

### Main Runtime Files

| File | Role |
|---|---|
| `kachaka_nav/nav_node.py` | Navigation action server with driver abstraction and path execution |
| `kachaka_nav/modular_nav.py` | Additional navigation server implementations |
| `kachaka_nav/modular_nav_rrt.py` | RRT-based navigation action server that reads YAML map metadata directly |
| `kachaka_nav/robot_driver.py` | Driver abstraction for real Kachaka hardware and ROS simulation |
| `kachaka_nav/coord_bridge_semantic.py` | Coordinate bridge between semantic and navigation frames |

### Planning and Utility Files

| File | Role |
|---|---|
| `kachaka_nav/rrt_lib.py` | Core RRT planning implementation for occupancy-map navigation |
| `kachaka_nav/rrt_lib_debug.py` | Debug-oriented RRT variant |
| `kachaka_nav/rrt_lib_old.py` | Older RRT implementation with different map assumptions |
| `kachaka_nav/align_maps.py` | Utility for comparing or aligning map metadata |
| `kachaka_nav/aligner_nudge.py` | Interactive or incremental map nudge tool |
| `kachaka_nav/calibrate.py` | Map calibration helper node |
| `kachaka_nav/click_to_nav.py` | Click-driven navigation utility |
| `kachaka_nav/get_coords.py` | Coordinate extraction helper |
| `kachaka_nav/verify_map.py` | Map validation and plotting tool |
| `kachaka_nav/vis.py` | Generates occupancy maps from semantic point clouds |
| `kachaka_nav/vis_map.py` | Robot-on-map visualization |
| `kachaka_nav/vis_kachaka.py` | Kachaka-specific visualization node |
| `kachaka_nav/npz2ply.py` | Converts semantic NPZ map output into PLY format |

### Navigation Architecture

The package is architected around two key abstractions:

1. `Navigate.action` as the motion contract.
2. `robot_driver.py` as the backend abstraction.

`robot_driver.py` provides:

- `KachakaRealDriver` for direct hardware access through the Kachaka API
- `RosSimDriver` for simulator-oriented velocity output on `/cmd_vel`

`nav_node.py` demonstrates the intended navigation flow:

- obtain the current pose from the active driver
- plan a route using RRT
- iterate through waypoints
- compute linear and angular velocity commands
- stop or cancel cleanly when required

This package is the motion execution backbone of the workspace.

<a id="section-47-map_alignment"></a>

## 4.7 `map_alignment`

### Purpose

`map_alignment` solves the calibration problem between a 3D semantic map and a 2D LiDAR or occupancy map. This is a critical subsystem because semantic object lookup is performed in a 3D reconstructed map, while robot navigation operates in a 2D motion-planning frame.

### Main Files

| File | Role |
|---|---|
| `map_alignment/map_alignment_v2.py` | Main offline-friendly alignment pipeline |
| `map_alignment/map_alignment.py` | Earlier, more extensive alignment implementation |
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

`object_query` exposes semantic map contents as a ROS 2 query service. It is the perception-facing semantic memory of the system.

### Main Files

| File | Role |
|---|---|
| `object_query/object_query_server.py` | Primary semantic object query service |
| `object_query/object_query_client.py` | Simple service client |
| `object_query/coord_bridge.py` | Coordinate bridge client |
| `object_query/object_query_server_2d3d.py` | Variant mixing 2D and 3D semantics |
| `object_query/object_query_server_kachaka.py` | Variant oriented toward Kachaka-specific use |
| `object_query/object_query_server_norm.py` | Normalized variant of the query server |
| `object_query/object_query_server_old.py` | Older minimal implementation |
| `object_query/object_query_test.py` | Expanded test or experimental server variant |
| `object_query/analyse_map.py` | Map redundancy analysis helper |
| `object_query/npz_read.py` | NPZ inspection helper |

### Core Behavior

The main server:

- loads semantic NPZ map data and associated metadata JSON
- optionally auto-aligns the semantic map by estimating a floor-normal rotation
- loads a separate 3D point cloud for visualization
- builds an in-memory database from category names to one or more object centroids
- provides the `object_query` service
- publishes object listings, queried markers, and the 3D map point cloud

The query logic:

- receives an object name
- searches the category-to-centroid database
- returns the nearest matching instance
- publishes RViz markers for the queried object only

This package is directly consumed by `decision_maker.WorldModel` and `DecisionMakingNode`.

<a id="section-49-object_query_interfaces"></a>

## 4.9 `object_query_interfaces`

### Purpose

This package defines the semantic lookup service contract.

### Interface File

| File | Description |
|---|---|
| `srv/ObjectQuery.srv` | Request object name, return found flag, 3D position, and message |

This service is the main interface between symbolic task commands and semantic map perception.

<a id="section-410-semantic_slam"></a>

## 4.10 `semantic_slam`

### Purpose

`semantic_slam` is an experimental or prototype package that simulates a semantic Gaussian-splatting SLAM action server.

### Main Files

| File | Role |
|---|---|
| `semantic_slam/run_slam_server.py` | Action server that generates synthetic incremental map data |
| `semantic_slam/run_slam_client.py` | Client for invoking the action |

### Core Behavior

`run_slam_server.py`:

- accepts a `RunSlam` action goal
- incrementally generates synthetic map arrays such as `means3D`, rotations, scales, opacities, RGB values, and semantic IDs
- appends these increments in memory
- publishes feedback containing the current number of splats
- optionally saves the resulting map as a compressed NPZ file when stopped or completed

This package is not a production SLAM implementation. It is a scaffold or interface prototype that demonstrates how a semantic SLAM subsystem could integrate with the rest of the workspace.

<a id="section-411-semantic_slam_interfaces"></a>

## 4.11 `semantic_slam_interfaces`

### Purpose

This package defines the custom action contract used by `semantic_slam`.

### Interface File

| File | Description |
|---|---|
| `action/RunSlam.action` | Start/stop semantic SLAM, return saved map URI and splat count |

<a id="section-5-custom-ros-interfaces-summary"></a>

## 5. Custom ROS Interfaces Summary

The workspace defines several custom interfaces that form the contract between packages:

| Package | Interface | Function |
|---|---|---|
| `kachaka_interfaces` | `Navigate.action` | Motion request to a target coordinate |
| `object_query_interfaces` | `ObjectQuery.srv` | Semantic lookup by object name |
| `decision_maker_interfaces` | `TaskCommand.action` | Generic task submission |
| `decision_maker_interfaces` | `Grasp.action` | Grasp-only action |
| `decision_maker_interfaces` | `Place.action` | Place-only action |
| `decision_maker_interfaces` | `GraspPlace.action` | Combined manipulation request |
| `semantic_slam_interfaces` | `RunSlam.action` | Semantic SLAM lifecycle request |

These interfaces are important because they decouple the packages cleanly. The orchestration logic does not need to know the internal implementation details of navigation, perception, or SLAM servers as long as these contracts are respected.

<a id="section-6-data-assets-and-utility-scripts"></a>

## 6. Data Assets and Utility Scripts

<a id="section-61-datautil"></a>

## 6.1 `data/Util`

The `data/Util/` directory contains conversion tools and baseline map assets used by semantic querying and alignment.

### Important Files

| File | Role |
|---|---|
| `Final_GS.npz` | 3D point-cloud map asset |
| `Final_SEM_GS_converted.npz` | Semantic map asset used by object-query nodes |
| `Final_SEM_GS_converted_meta.json` | Metadata describing semantic segments/categories |
| `alignment.yaml` | Main 3D-to-2D alignment file consumed by `decision_maker` |
| `alignment_modified.yaml` | Additional alignment variant |
| `alignment_origin.yaml` | Original alignment variant |
| `semantic_legend.json` | Semantic category/color reference |
| `actual_color_mapping.json` | Color mapping asset |

### Utility Scripts

| File | Role |
|---|---|
| `ply_to_npz.py` | Converts PLY point clouds to NPZ |
| `read_ply.py` | Reads PLY point clouds |
| `read_npz.py` | Reads NPZ map files |
| `downsample.py` | Downsampling utility |
| `convert_sem_gs_exact.py` | Converts semantic Gaussian-splatting outputs |
| `convert_sem_gs_rgb_ply.py` | Converts semantic map data into RGB PLY |
| `create_semantic_meta.py` | Builds semantic metadata JSON |
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
| `align.py` | Alignment helper |
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
- `WorldModel` resolves places and objects.

### 7.3 Semantic Lookup

- `WorldModel` or `DecisionMakingNode` calls `ObjectQuery.srv`.
- `object_query_server.py` returns a centroid in the semantic 3D map frame.

### 7.4 Frame Conversion

- `decision_maker_node.py` loads `data/Util/alignment.yaml`.
- the returned 3D object location is projected and transformed into the 2D navigation frame.

### 7.5 Navigation Execution

- `decision_maker_node.py` sends a `Navigate` action goal.
- `kachaka_nav` receives the action and plans a path.
- `robot_driver.py` dispatches low-level velocity or robot-native commands.

### 7.6 Manipulation Execution

- `decision_maker_node.py` sends `GraspPlace` or related actions.
- mock servers can emulate this during testing.

### 7.7 Visualization and Debugging

- `object_query` publishes queried markers and point clouds.
- `decision_maker` can show target markers on a 2D map via OpenCV.
- `get_pose` can republish TF as `PoseStamped`.
- `kachaka_laser_api` publishes `LaserScan` for sensing and debugging.

This interaction pattern confirms that the workspace is not a collection of isolated experiments; it is an integrated semantic-task robotics stack centered on ROS 2 message passing and action/service composition.

<a id="section-8-testing-and-quality-support"></a>

## 8. Testing and Quality Support

Most Python packages contain standard ROS 2 lint-style tests:

- `test_flake8.py`
- `test_pep257.py`
- `test_copyright.py`

These files indicate baseline attention to packaging and style compliance. However, the workspace appears to rely more heavily on integration-oriented scripts, experimental variants, and runtime testing than on extensive unit-test coverage of business logic.

<a id="section-9-observations-on-codebase-maturity"></a>

## 9. Observations on Codebase Maturity

Several characteristics are visible across the repository:

### Strengths

- The workspace is modular and uses ROS 2 interfaces appropriately.
- Package responsibilities are generally well separated.
- Semantic querying, alignment, and navigation are connected into a coherent pipeline.
- The repository includes practical utilities for data conversion and environment preparation.
- Launch files and mock servers support incremental testing.

### Signs of Ongoing Development

- Many `package.xml` files still contain placeholder descriptions and license fields.
- Several packages contain multiple experimental variants such as `_old`, `_norm`, `_test`, and simulator-specific nodes.
- `requirements.txt` is a full environment snapshot, which is useful for reproduction but less useful for long-term dependency maintenance.
- The codebase mixes production-oriented nodes with research or prototype scripts.

Overall, the workspace should be understood as an actively evolving robotics development environment rather than a finalized product repository.

<a id="section-10-key-files-worth-immediate-attention"></a>

## 10. Key Files Worth Immediate Attention

For any engineer onboarding to this workspace, the most important files to read first are:

1. `src/decision_maker/decision_maker/decision_maker_node.py`
2. `src/decision_maker/decision_maker/scenario_library.py`
3. `src/decision_maker/decision_maker/nl_planner.py`
4. `src/object_query/object_query/object_query_server.py`
5. `src/kachaka_nav/kachaka_nav/nav_node.py`
6. `src/kachaka_nav/kachaka_nav/robot_driver.py`
7. `src/map_alignment/map_alignment/map_alignment_v2.py`
8. `data/Util/alignment.yaml`
9. `src/object_query_interfaces/srv/ObjectQuery.srv`
10. `src/kachaka_interfaces/action/Navigate.action`

These files capture the core task pipeline, inter-package contracts, and frame-conversion logic.

<a id="section-11-conclusion"></a>

## 11. Conclusion

`robot_ws` is a ROS 2 robotics workspace built around semantic task execution in a mapped environment. Its architecture combines command understanding, semantic map querying, frame alignment, navigation, and hardware integration. The most important technical idea in the repository is the bridge between semantic 3D reconstruction outputs and 2D robot navigation, implemented through `object_query`, `map_alignment`, `decision_maker`, and `kachaka_nav`.

The repository also contains significant supporting infrastructure: data-conversion tools, environment-specific datasets, generated build artifacts, mock action servers, visualization helpers, and prototype SLAM interfaces. As a result, this workspace serves both as an application stack for semantic mobile manipulation experiments and as a development platform for robotics research and system integration.

<a id="section-12-practical-setup-and-execution-guide"></a>

## 12. Practical Setup and Execution Guide

This section adds an operator-oriented guide for building the environment and launching the key packages discussed above. The commands below are based on the current repository layout under `/robotic-project`.

### 12.1 Path Assumptions

The instructions in this section assume:

- project root: `/robotic-project`
- ROS 2 workspace root: `/robotic-project/robot_ws`
- ROS distribution: `humble`
- Conda environment name: `robot_ros`

Because several packages use relative default paths such as `data/Util/alignment.yaml` and `data/lab/kachaka_native.yaml`, it is safer to either:

- run commands from the workspace root, or
- pass absolute paths through `--ros-args -p ...`

The examples below use absolute paths to avoid ambiguity.

### 12.2 Local Conda Environment Setup

The repository already includes:

- `/robotic-project/robot_ws/environment.yml`
- `/robotic-project/robot_ws/requirements.txt`
- `/robotic-project/Dockerfile`

The Dockerfile is the most complete reference for the intended Python environment. A local Conda workflow equivalent to that Dockerfile is shown below.

#### 12.2.1 Create and Activate the Environment

```bash
cd /robotic-project/robot_ws

source "$(conda info --base)/etc/profile.d/conda.sh"
conda env create -f environment.yml || conda create -n robot_ros python=3.10 -y
conda activate robot_ros
```

#### 12.2.2 Install Python Tooling Used by the Workspace

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

#### 12.2.3 Install Packages That the Dockerfile Installs Through Conda

The Dockerfile installs several packages through Conda rather than `pip`, mainly for compatibility on ARM and robotics systems:

```bash
conda install -n robot_ros -c conda-forge -y grpcio=1.59.3
conda install -n robot_ros -c conda-forge -y open3d
conda install -n robot_ros -c conda-forge -y pyrealsense2 || true
```

#### 12.2.4 Install the Workspace Python Requirements

```bash
pip install -r /robotic-project/robot_ws/requirements.txt
```

#### 12.2.5 Make ROS Python Packages Visible Inside the Conda Environment

The Dockerfile adds ROS Humble site-packages into the Conda environment through a `.pth` file. The same idea can be applied locally:

```bash
python - <<'PY'
import sysconfig
from pathlib import Path

purelib = Path(sysconfig.get_paths()["purelib"])
(purelib / "ros_humble_local.pth").write_text("/opt/ros/humble/lib/python3.10/site-packages\n")
PY
```

If the local ROS installation uses a different Python minor version, adjust the path accordingly.

#### 12.2.6 Build the Workspace

```bash
cd /robotic-project/robot_ws
source /opt/ros/humble/setup.bash
conda activate robot_ros
colcon build --merge-install --symlink-install
source install/setup.bash
```

#### 12.2.7 Recommended Shell Initialization for Every New Terminal

```bash
cd /robotic-project/robot_ws
source /opt/ros/humble/setup.bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate robot_ros
source install/setup.bash
export PYTHONPATH=$PYTHONPATH:/opt/conda/envs/robot_ros/lib/python3.10/site-packages
```

If the environment is local rather than container-based, replace `/opt/conda/envs/robot_ros/...` with the local Conda environment path shown by `conda info --envs`.

### 12.3 Docker Environment Setup

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

#### 12.3.1 Build the Docker Image

```bash
cd /robotic-project
docker build -t robotic-project:humble .
```

#### 12.3.2 Run the Container for Development

For terminal-only use:

```bash
docker run --rm -it \
  --net=host \
  --ipc=host \
  --privileged \
  -v /robotic-project/robot_ws:/robot_ws \
  robotic-project:humble
```

For GUI-based tools such as OpenCV windows or RViz:

```bash
xhost +local:root

docker run --rm -it \
  --net=host \
  --ipc=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  -v /robotic-project/robot_ws:/robot_ws \
  robotic-project:humble
```

After the container starts, the entrypoint should already leave the shell in a usable state. If needed, re-run:

```bash
source /opt/ros/humble/setup.bash
source /opt/conda/etc/profile.d/conda.sh
conda activate robot_ros
cd /robot_ws
source install/setup.bash
```

### 12.4 Workspace Build Commands

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

### 12.5 Package Execution Commands

This subsection provides the practical `ros2 run` commands for the packages the repository uses most often in the semantic-task pipeline.

### 12.5.1 Start `object_query`

This node loads the semantic map, publishes the point cloud and queried markers, and exposes the `ObjectQuery` service.

```bash
cd /robotic-project/robot_ws
source /opt/ros/humble/setup.bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate robot_ros
source install/setup.bash

ros2 run object_query object_query_server --ros-args \
  -p 3dmap_path:=/robotic-project/robot_ws/data/Util/Final_GS.npz \
  -p map_path:=/robotic-project/robot_ws/data/Util/Final_SEM_GS_converted.npz \
  -p semantic_path:=/robotic-project/robot_ws/data/Util/Final_SEM_GS_converted_meta.json \
  -p auto_align:=false
```

Optional client test:

```bash
ros2 run object_query object_query_client chair
```

### 12.5.2 Start `kachaka_nav`

The package exports several executables:

- `nav_node`
- `modular_nav_node`
- `modular_nav_node_rrt`
- `coord_bridge`

For direct navigation experiments, the most self-contained example in the current codebase is `modular_nav_node_rrt`, because it reads a Kachaka map YAML file and then derives the resolution and origin internally.

#### Real Robot Example

```bash
cd /robotic-project/robot_ws
source /opt/ros/humble/setup.bash
source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate robot_ros
source install/setup.bash

ros2 run kachaka_nav modular_nav_node_rrt --ros-args \
  -p use_sim:=false \
  -p kachaka_ip:=192.168.0.157:26400 \
  -p use_native_map:=true
```

#### Simulation Example

```bash
ros2 run kachaka_nav modular_nav_node_rrt --ros-args \
  -p use_sim:=true \
  -p use_native_map:=false
```

#### Alternate `nav_node` Example

If you want to invoke the executable registered as `nav_node`, pass all planning inputs explicitly:

```bash
ros2 run kachaka_nav nav_node --ros-args \
  -p use_sim:=false \
  -p kachaka_ip:=192.168.0.157:26400 \
  -p map_path:=/robotic-project/robot_ws/data/lab/kachaka_native.png \
  -p meta_path:=/robotic-project/robot_ws/data/lab/kachaka_native.yaml \
  -p points_path:=/robotic-project/robot_ws/data/lab/semantic_pcd_accumulated_gaussians.npz
```

### 12.5.3 Start `decision_maker`

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

Other useful executables from the same package:

```bash
ros2 run decision_maker decision_maker_node_calib
ros2 run decision_maker decision_maker_node_isaacsim
ros2 run decision_maker decision_maker_test
ros2 run decision_maker text_command_node
ros2 run decision_maker audio_command_node
ros2 run decision_maker cancel_command_node
```

If you only want to validate the orchestration flow without real downstream servers, the package also provides mocks:

```bash
ros2 launch decision_maker mock_actions.launch.py
```

### 12.5.4 Start `nl_command_node`

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
give me apple
go home
```

### 12.5.5 Optional Supporting Nodes

#### LiDAR from Kachaka API

```bash
ros2 run kachaka_laser_api laser_from_api --ros-args \
  -p kachaka_ip:=192.168.0.157:26400 \
  -p topic_name:=/scan \
  -p rate_hz:=5.0
```

#### TF to Pose

```bash
ros2 run get_pose tf_to_pose --ros-args \
  -p target_frame:=map \
  -p base_frame:=base_footprint \
  -p rate_hz:=20.0
```

#### Semantic SLAM Prototype

```bash
ros2 run semantic_slam run_slam_server
ros2 run semantic_slam run_slam_client
```

### 12.6 Recommended Multi-Terminal Launch Order

For an integrated manual test of the semantic navigation stack, open several terminals and launch the nodes in the following order.

#### Terminal 1: Semantic Query Service

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
ros2 run kachaka_nav modular_nav_node_rrt --ros-args \
  -p use_sim:=false \
  -p kachaka_ip:=192.168.0.157:26400 \
  -p use_native_map:=true
```

#### Terminal 3: Decision Layer

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

### 12.7 Notes and Operational Caveats

- The repository currently mixes several navigation and decision-maker variants. The commands above are based on the executables registered in the current `setup.py` files.
- Relative file defaults in the source code can fail if the node is launched from another directory. Absolute parameter paths are safer.
- The Dockerfile and entrypoint are currently the most complete description of the intended environment.
- If OpenCV windows or RViz are required, X11 forwarding and device access must be enabled for Docker.
- Real robot operation requires valid Kachaka network connectivity and, for sensor-related nodes, the necessary device permissions.
