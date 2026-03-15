# Robotit Project Workspace Description

<a id="table-of-contents"></a>

## Table of Contents

- [1. Executive Summary](#section-1-executive-summary)
- [2. Repository Structure](#section-2-repository-structure)
- [3. Environment and Dependency Configuration](#section-3-environment-and-dependency-configuration)
- [4. Package Descriptions](#section-4-package-by-package-technical-analysis)
- [4.1 `decision_maker`](#section-41-decision_maker)
- [4.2 `decision_maker_interfaces`](#section-42-decision_maker_interfaces)
- [4.3 `get_pose`](#section-43-get_pose)
- [4.4 `kachaka_interfaces`](#section-44-kachaka_interfaces)
- [4.5 `kachaka_laser_api`](#section-45-kachaka_laser_api)
- [4.6 `kachaka_nav`](#section-46-kachaka_nav)
- [4.7 `map_alignment`](#section-47-map_alignment)
- [4.8 `object_query`](#section-48-object_query)
- [4.9 `object_query_interfaces`](#section-49-object_query_interfaces)
- [4.8 `semantic_slam`](#section-48-semantic_slam)
- [4.8 `semantic_slam_interfaces`](#section-48-semantic_slam_interfaces)
- [5. Custom ROS Interfaces Summary](#section-5-custom-ros-interfaces-summary)
- [6. Data Assets and Utility Scripts](#section-6-data-assets-and-utility-scripts)
- [6.1 `data/Util`](#section-61-datautil)
- [6.2 `data/lab`](#section-62-datalab)
- [7. Runtime Interaction Between Packages](#section-7-runtime-interaction-between-packages)
- [8. Practical Setup and Execution Guide](#section-8-practical-setup-and-execution-guide)

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

`decision_maker` is the central orchestration package. It receives human commands, transforms them into primitive actions, queries semantic object locations, converts those coordinates into the navigation frame, and dispatches navigation and manipulation requests to downstream servers.

### Main Files

| File | Role |
|---|---|
| `decision_maker/decision_maker_node.py` | Primary task-execution node for the integrated system |
| `decision_maker/decision_maker_test.py` | Experimental or test-oriented variant of the orchestration node |
| `decision_maker/text_command_node.py` | Reads terminal input and publishes text commands |
| `decision_maker/audio_command_node.py` | Records audio, runs speech-to-text, and publishes commands |
| `decision_maker/nl_command_node.py` | Natural-language command publisher node |
| `decision_maker/cancel_command_node.py` | Publishes cancellation requests |
| `decision_maker/mock_nav_server.py` | Mock navigation action server for testing |
| `decision_maker/mock_grasp_server.py` | Mock grasp/place action server for testing |
| `decision_maker/scenario_library.py` | Maps human phrases to multi-step task sequences |
| `decision_maker/nl_planner.py` | World model and command-to-primitive formatting utilities |
| `decision_maker/command_types.py` | Command data abstraction |

### Core Behavior

The central implementation is `decision_maker_node.py`. Its responsibilities are:

- subscribe to `/manual_command` and `/cancel_command`
- maintain a bounded command queue
- use `scenario_library.py` to parse high-level tasks such as `go to`, `bring`, `give me`, `fetch drink`, and `clean table`
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

`nl_planner.py` bridges semantic queries and static place definitions:

- static places such as `me` and `home` are resolved locally
- unknown places are forwarded to the `ObjectQuery` service
- objects are always resolved through the semantic map service
- `fmt_goto()` formats pose tuples into the primitive `goto:` command syntax expected by the executor

<a id="section-42-decision_maker_interfaces"></a>

## 4.2 `decision_maker_interfaces`

### Purpose

This package defines custom ROS 2 actions used for manipulation and task dispatch.

### Interface Files

| File | Description |
|---|---|
| `action/TaskCommand.action` | Generic text task action |

The action provides the type contracts required by nodes in `decision_maker`.

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

`kachaka_nav` provides navigation action servers, path planning libraries, robot drivers, coordinate conversion helpers, and several map visualization and calibration utilities.

### Main Runtime Files

| File | Role |
|---|---|
| `kachaka_nav/modular_nav.py` | Additional navigation server implementations |
| `kachaka_nav/robot_driver.py` | Driver abstraction for real Kachaka hardware and ROS simulation |

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
| `object_query/object_query_test.py` | Expanded test or experimental server variant |

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

<a id="section-49-object_query_interfaces"></a>

## 4.9 `object_query_interfaces`

### Purpose

This package defines the semantic lookup service contract.

### Interface File

| File | Description |
|---|---|
| `srv/ObjectQuery.srv` | Request object name, return found flag, 3D position, and message |

This service is the main interface between symbolic task commands and semantic map perception.

<a id="section-48-semantic_slam"></a>


<a id="section-5-custom-ros-interfaces-summary"></a>

## 5. Custom ROS Interfaces Summary

The workspace defines several custom interfaces that form the contract between packages:

| Package | Interface | Function |
|---|---|---|
| `kachaka_interfaces` | `Navigate.action` | Motion request to a target coordinate |
| `object_query_interfaces` | `ObjectQuery.srv` | Semantic lookup by object name |
| `decision_maker_interfaces` | `TaskCommand.action` | Generic task submission |
| `decision_maker_interfaces` | `GraspPlace.action` | Combined manipulation request |
| `semantic_slam_interfaces` | `RunSlam.action` | Semantic SLAM lifecycle request |

These interfaces are important because they decouple the packages cleanly. The orchestration logic does not need to know the internal implementation details of navigation, perception, or SLAM servers as long as these contracts are respected.

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

- project root: `/robotic_system_`
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
export PYTHONPATH=$PYTHONPATH:/opt/conda/envs/robot_ros/lib/python3.8/site-packages
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

