FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rviz2 \
    ros-humble-desktop \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-rosbag2-transport \
    git curl python3-pip \
    git nano \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-rmw-cyclonedds-cpp \
    python3-venv python3-pip \
    build-essential cmake git curl \
    ros-humble-ackermann-msgs \
    pkg-config \
    libssl-dev \
    libprotobuf-dev \
    protobuf-compiler \
    zlib1g-dev \
    python3-dev \
    python3-setuptools \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    wget \
    bzip2 \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    && rm -rf /var/lib/apt/lists/*

# 安裝編譯工具和依賴
RUN apt-get update && apt-get install -y --no-install-recommends \
    git cmake libusb-1.0-0-dev pkg-config libgtk-3-dev \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

# 下載 librealsense source
RUN git clone https://github.com/IntelRealSense/librealsense.git /tmp/librealsense \
    && cd /tmp/librealsense \
    && mkdir build && cd build \
    && cmake .. \
        -DBUILD_EXAMPLES=false \
        -DBUILD_GRAPHICAL_EXAMPLES=false \
        -DBUILD_WITH_TM2=false \
    && make -j$(nproc) \
    && make install \
    && ldconfig \
    && rm -rf /tmp/librealsense

RUN wget -q https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh \
    -O /tmp/miniconda.sh && \
    bash /tmp/miniconda.sh -b -p /opt/conda && \
    rm /tmp/miniconda.sh

ENV PATH=/opt/conda/bin:$PATH
# ENV PYTHONPATH=$PYTHONPATH:/opt/conda/envs/robot_ros/lib/python3.10/site-packages
# rosdep
RUN rosdep init || true
RUN rosdep update

WORKDIR /robot_ws
COPY robot_ws ./robot_ws

RUN conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main \
 && conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r

RUN conda env create -f ./robot_ws/environment.yml

ENV PATH=/opt/conda/envs/robot_ros/bin:$PATH
# ENV PYTHONPATH=/opt/conda/envs/robot_ros/lib/python3.10/site-packages:$PYTHONPATH
# Use the conda env for all Python installs and build the workspace with the
# conda Python so console entrypoints reference the conda interpreter.
# Ensure the conda env can import ROS Python packages by adding ROS site-packages
# to the env's site-packages via a .pth file (adjust python minor version if needed).
RUN python_site_pkgs="$(python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')" \
 && echo "/opt/ros/humble/lib/python3.10/site-packages" > /opt/conda/envs/robot_ros/lib/python3.10/site-packages/ros_humble.pth \
 && echo "/robot_ws/install" > /opt/conda/envs/robot_ros/lib/python3.10/site-packages/robot_ws_install.pth || true

# Upgrade pip/setuptools/wheel inside the conda env and install Cython there.
RUN /opt/conda/bin/conda run -n robot_ros pip install --upgrade pip setuptools wheel
RUN /opt/conda/bin/conda run -n robot_ros pip install Cython

# Install grpcio into the conda env from conda-forge (prebuilt wheel for aarch64)
RUN /opt/conda/bin/conda install -n robot_ros -c conda-forge -y grpcio=1.59.3

# Install kachaka-api into the conda env
RUN /opt/conda/bin/conda run -n robot_ros pip install --no-cache-dir kachaka-api==3.14.4.0
# RUN rosdep install --from-paths src --ignore-src -r -y

# Install Open3D via conda (conda-forge) because pip may not have a prebuilt
# aarch64 wheel; installing via conda avoids the "No matching distribution" error
# when running `pip install -r ./robot_ws/requirement.txt` below.
RUN /opt/conda/bin/conda install -n robot_ros -c conda-forge -y open3d

# Install Python requirements into the conda env (use conda when possible).
RUN /opt/conda/bin/conda run -n robot_ros pip install -r ./robot_ws/requirement.txt || true

# Try installing pyrealsense2 via conda-forge into the env (may not exist)
RUN /opt/conda/bin/conda install -n robot_ros -c conda-forge -y pyrealsense2 || true

# Install remaining small packages into the conda env
RUN /opt/conda/bin/conda run -n robot_ros pip install --no-cache-dir pyyaml tqdm transforms3d || true

# Build the ROS workspace inside the conda env so installed console scripts
# and Python packages use the conda Python interpreter.
## Ensure ROS/ament build-time Python packages are available inside the conda env
## (e.g. catkin_pkg is required by ament's package_xml_2_cmake.py)
RUN /opt/conda/bin/conda run -n robot_ros pip uninstall -y em || true
RUN /opt/conda/bin/conda run -n robot_ros pip install --no-cache-dir -U catkin_pkg lxml || true
RUN /opt/conda/bin/conda run -n robot_ros pip install --no-cache-dir empy==3.3.4 || true
## Install parser dependency required by rosidl (lark)
RUN /opt/conda/bin/conda run -n robot_ros pip install --no-cache-dir lark-parser || true

# Now build the workspace inside the conda env
RUN /bin/bash -lc "source /opt/conda/etc/profile.d/conda.sh && conda activate robot_ros && source /opt/ros/humble/setup.bash && cd /robot_ws && colcon build --merge-install --symlink-install"

WORKDIR /robot_ws
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/bin/bash", "-c", "/entrypoint.sh"]
