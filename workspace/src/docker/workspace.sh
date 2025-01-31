#!/bin/bash
set -e

# Set ROS 2 distribution as a variable
ROS_DISTRO="jazzy"

# Source ROS 2 setup
source /opt/ros/$ROS_DISTRO/setup.bash

# Install system dependencies for MongoDB and PCL
#apt-get update && apt-get install -y \
#    gnupg \
#    curl \
#    libpcap-dev
#
## Install MongoDB (following official MongoDB installation for Ubuntu)
#curl -fsSL https://www.mongodb.org/static/pgp/server-8.0.asc | \
#    gpg -o /usr/share/keyrings/mongodb-server-8.0.gpg \
#    --dearmor
#echo "deb [ arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server-8.0.gpg ] https://repo.mongodb.org/apt/ubuntu noble/mongodb-org/8.0 multiverse" | \
#    tee /etc/apt/sources.list.d/mongodb-org-8.0.list
#apt-get update && apt-get install -y mongodb-org
#
## Start and enable MongoDB service
## Note: systemctl might not work in all Docker environments, so we'll add error handling
#systemctl start mongod || echo "Warning: Could not start MongoDB service. This is expected in some Docker environments."
#systemctl enable mongod || echo "Warning: Could not enable MongoDB service. This is expected in some Docker environments."
#
## Navigate to the workspace
#cd /root/ros2_ws/src
#
## Install warehouse_ros_mongo if not already present
#if [ ! -d "warehouse_ros_mongo" ]; then
#    git clone https://github.com/moveit/warehouse_ros_mongo.git -b ros2
#    cd warehouse_ros_mongo/
#    git reset --hard 32f8fc5dd245077b9c09e93efc8625b9f599f271
#    cd ..
#fi
#
## Install MoveIt Task Constructor if not already present
#if [ ! -d "moveit_task_constructor" ]; then
#    git clone https://github.com/moveit/moveit_task_constructor.git -b jazzy
#    cd moveit_task_constructor
#    git reset --hard 9ced9fc10a15388224f0741e5a930a33f4ed6255
#    cd ..
#fi

# Install core dependencies
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_GB en_GB.UTF-8
sudo update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8
export LANG=en_GB.UTF-8
export LC_NUMERIC="en_GB.UTF-8"

locale  # verify settings


sudo apt update && sudo apt install -y ros-dev-tools

sudo apt install -y python3-colcon-argcomplete \
ros-jazzy-ackermann-msgs \
ros-jazzy-ackermann-steering-controller \
ros-jazzy-action-msgs \
ros-jazzy-action-tutorials-cpp \
ros-jazzy-action-tutorials-interfaces \
ros-jazzy-action-tutorials-py \
ros-jazzy-actionlib-msgs \
ros-jazzy-actuator-msgs \
ros-jazzy-admittance-controller \
ros-jazzy-ament-cmake-auto \
ros-jazzy-ament-cmake-copyright \
ros-jazzy-ament-cmake-core \
ros-jazzy-ament-cmake-cppcheck \
ros-jazzy-ament-cmake-cpplint \
ros-jazzy-ament-cmake-export-definitions \
ros-jazzy-ament-cmake-export-dependencies \
ros-jazzy-ament-cmake-export-include-directories \
ros-jazzy-ament-cmake-export-interfaces \
ros-jazzy-ament-cmake-export-libraries \
ros-jazzy-ament-cmake-export-link-flags \
ros-jazzy-ament-cmake-export-targets \
ros-jazzy-ament-cmake-flake8 \
ros-jazzy-ament-cmake-gen-version-h \
ros-jazzy-ament-cmake-gmock \
ros-jazzy-ament-cmake-gtest \
ros-jazzy-ament-cmake-include-directories \
ros-jazzy-ament-cmake-libraries \
ros-jazzy-ament-cmake-lint-cmake \
ros-jazzy-ament-cmake-pep257 \
ros-jazzy-ament-cmake-pytest \
ros-jazzy-ament-cmake-python \
ros-jazzy-ament-cmake-ros \
ros-jazzy-ament-cmake-target-dependencies \
ros-jazzy-ament-cmake-test \
ros-jazzy-ament-cmake-uncrustify \
ros-jazzy-ament-cmake-version \
ros-jazzy-ament-cmake-xmllint \
ros-jazzy-ament-cmake \
ros-jazzy-ament-copyright \
ros-jazzy-ament-cppcheck \
ros-jazzy-ament-cpplint \
ros-jazzy-ament-flake8 \
ros-jazzy-ament-index-cpp \
ros-jazzy-ament-index-python \
ros-jazzy-ament-lint-auto \
ros-jazzy-ament-lint-cmake \
ros-jazzy-ament-lint-common \
ros-jazzy-ament-lint \
ros-jazzy-ament-package \
ros-jazzy-ament-pep257 \
ros-jazzy-ament-uncrustify \
ros-jazzy-ament-xmllint \
ros-jazzy-angles \
ros-jazzy-backward-ros \
ros-jazzy-bicycle-steering-controller \
ros-jazzy-builtin-interfaces \
ros-jazzy-chomp-motion-planner \
ros-jazzy-class-loader \
ros-jazzy-common-interfaces \
ros-jazzy-composition-interfaces \
ros-jazzy-composition \
ros-jazzy-compressed-depth-image-transport \
ros-jazzy-compressed-image-transport \
ros-jazzy-console-bridge-vendor \
ros-jazzy-control-msgs \
ros-jazzy-control-toolbox \
ros-jazzy-controller-interface \
ros-jazzy-controller-manager-msgs \
ros-jazzy-controller-manager \
ros-jazzy-cv-bridge \
ros-jazzy-cyclonedds \
ros-jazzy-demo-nodes-cpp-native \
ros-jazzy-demo-nodes-cpp \
ros-jazzy-demo-nodes-py \
ros-jazzy-depthimage-to-laserscan \
ros-jazzy-desktop \
ros-jazzy-diagnostic-msgs \
ros-jazzy-diagnostic-updater \
ros-jazzy-diff-drive-controller \
ros-jazzy-domain-coordinator \
ros-jazzy-dummy-map-server \
ros-jazzy-dummy-robot-bringup \
ros-jazzy-dummy-sensors \
ros-jazzy-effort-controllers \
ros-jazzy-eigen-stl-containers \
ros-jazzy-eigen3-cmake-module \
ros-jazzy-example-interfaces \
ros-jazzy-examples-rclcpp-minimal-action-client \
ros-jazzy-examples-rclcpp-minimal-action-server \
ros-jazzy-examples-rclcpp-minimal-client \
ros-jazzy-examples-rclcpp-minimal-composition \
ros-jazzy-examples-rclcpp-minimal-publisher \
ros-jazzy-examples-rclcpp-minimal-service \
ros-jazzy-examples-rclcpp-minimal-subscriber \
ros-jazzy-examples-rclcpp-minimal-timer \
ros-jazzy-examples-rclcpp-multithreaded-executor \
ros-jazzy-examples-rclpy-executors \
ros-jazzy-examples-rclpy-minimal-action-client \
ros-jazzy-examples-rclpy-minimal-action-server \
ros-jazzy-examples-rclpy-minimal-client \
ros-jazzy-examples-rclpy-minimal-publisher \
ros-jazzy-examples-rclpy-minimal-service \
ros-jazzy-examples-rclpy-minimal-subscriber \
ros-jazzy-fastcdr \
ros-jazzy-fastrtps-cmake-module \
ros-jazzy-fastrtps \
ros-jazzy-filters \
ros-jazzy-foonathan-memory-vendor \
ros-jazzy-force-torque-sensor-broadcaster \
ros-jazzy-forward-command-controller \
ros-jazzy-generate-parameter-library-py \
ros-jazzy-generate-parameter-library \
ros-jazzy-geometric-shapes \
ros-jazzy-geometry-msgs \
ros-jazzy-geometry2 \
ros-jazzy-gmock-vendor \
ros-jazzy-google-benchmark-vendor \
ros-jazzy-gpio-controllers \
ros-jazzy-gps-msgs \
ros-jazzy-graph-msgs \
ros-jazzy-grbl-msgs \
ros-jazzy-grbl-ros \
ros-jazzy-gripper-controllers \
ros-jazzy-gtest-vendor \
ros-jazzy-gz-cmake-vendor \
ros-jazzy-gz-common-vendor \
ros-jazzy-gz-dartsim-vendor \
ros-jazzy-gz-fuel-tools-vendor \
ros-jazzy-gz-gui-vendor \
ros-jazzy-gz-math-vendor \
ros-jazzy-gz-msgs-vendor \
ros-jazzy-gz-ogre-next-vendor \
ros-jazzy-gz-physics-vendor \
ros-jazzy-gz-plugin-vendor \
ros-jazzy-gz-rendering-vendor \
ros-jazzy-gz-ros2-control-dbgsym \
ros-jazzy-gz-ros2-control-demos-dbgsym \
ros-jazzy-gz-ros2-control-demos \
ros-jazzy-gz-ros2-control \
ros-jazzy-gz-sensors-vendor \
ros-jazzy-gz-sim-vendor \
ros-jazzy-gz-tools-vendor \
ros-jazzy-gz-transport-vendor \
ros-jazzy-gz-utils-vendor \
ros-jazzy-hardware-interface \
ros-jazzy-iceoryx-binding-c \
ros-jazzy-iceoryx-hoofs \
ros-jazzy-iceoryx-posh \
ros-jazzy-image-geometry \
ros-jazzy-image-tools \
ros-jazzy-image-transport-plugins \
ros-jazzy-image-transport \
ros-jazzy-imu-sensor-broadcaster \
ros-jazzy-interactive-markers \
ros-jazzy-intra-process-demo \
ros-jazzy-joint-limits \
ros-jazzy-joint-state-broadcaster \
ros-jazzy-joint-state-publisher-gui \
ros-jazzy-joint-state-publisher \
ros-jazzy-joint-trajectory-controller \
ros-jazzy-joy \
ros-jazzy-kdl-parser \
ros-jazzy-keyboard-handler \
ros-jazzy-kinematics-interface \
ros-jazzy-laser-geometry \
ros-jazzy-launch-param-builder \
ros-jazzy-launch-ros \
ros-jazzy-launch-testing-ament-cmake \
ros-jazzy-launch-testing-ros \
ros-jazzy-launch-testing \
ros-jazzy-launch-xml \
ros-jazzy-launch-yaml \
ros-jazzy-launch \
ros-jazzy-libcurl-vendor \
ros-jazzy-liblz4-vendor \
ros-jazzy-libstatistics-collector \
ros-jazzy-libyaml-vendor \
ros-jazzy-lifecycle-msgs \
ros-jazzy-lifecycle \
ros-jazzy-logging-demo \
ros-jazzy-map-msgs \
ros-jazzy-mcap-vendor \
ros-jazzy-mecanum-drive-controller \
ros-jazzy-message-filters \
ros-jazzy-moveit-common \
ros-jazzy-moveit-configs-utils \
ros-jazzy-moveit-core-dbgsym \
ros-jazzy-moveit-core \
ros-jazzy-moveit-hybrid-planning-dbgsym \
ros-jazzy-moveit-hybrid-planning \
ros-jazzy-moveit-kinematics-dbgsym \
ros-jazzy-moveit-kinematics \
ros-jazzy-moveit-msgs-dbgsym \
ros-jazzy-moveit-msgs \
ros-jazzy-moveit-planners-chomp-dbgsym \
ros-jazzy-moveit-planners-chomp \
ros-jazzy-moveit-planners-ompl-dbgsym \
ros-jazzy-moveit-planners-ompl \
ros-jazzy-moveit-planners-stomp-dbgsym \
ros-jazzy-moveit-planners-stomp \
ros-jazzy-moveit-planners \
ros-jazzy-moveit-plugins \
ros-jazzy-moveit-py-dbgsym \
ros-jazzy-moveit-py \
ros-jazzy-moveit-resources-fanuc-description \
ros-jazzy-moveit-resources-fanuc-moveit-config \
ros-jazzy-moveit-resources-panda-description \
ros-jazzy-moveit-resources-panda-moveit-config \
ros-jazzy-moveit-resources-pr2-description \
ros-jazzy-moveit-resources-prbt-ikfast-manipulator-plugin-dbgsym \
ros-jazzy-moveit-resources-prbt-ikfast-manipulator-plugin \
ros-jazzy-moveit-resources-prbt-moveit-config \
ros-jazzy-moveit-resources-prbt-pg70-support \
ros-jazzy-moveit-resources-prbt-support \
ros-jazzy-moveit-resources \
ros-jazzy-moveit-ros-benchmarks-dbgsym \
ros-jazzy-moveit-ros-benchmarks \
ros-jazzy-moveit-ros-control-interface-dbgsym \
ros-jazzy-moveit-ros-control-interface \
ros-jazzy-moveit-ros-move-group-dbgsym \
ros-jazzy-moveit-ros-move-group \
ros-jazzy-moveit-ros-occupancy-map-monitor-dbgsym \
ros-jazzy-moveit-ros-occupancy-map-monitor \
ros-jazzy-moveit-ros-perception-dbgsym \
ros-jazzy-moveit-ros-perception \
ros-jazzy-moveit-ros-planning-dbgsym \
ros-jazzy-moveit-ros-planning-interface-dbgsym \
ros-jazzy-moveit-ros-planning-interface \
ros-jazzy-moveit-ros-planning \
ros-jazzy-moveit-ros-robot-interaction-dbgsym \
ros-jazzy-moveit-ros-robot-interaction \
ros-jazzy-moveit-ros-tests \
ros-jazzy-moveit-ros-visualization-dbgsym \
ros-jazzy-moveit-ros-visualization \
ros-jazzy-moveit-ros-warehouse-dbgsym \
ros-jazzy-moveit-ros-warehouse \
ros-jazzy-moveit-ros \
ros-jazzy-moveit-runtime \
ros-jazzy-moveit-servo-dbgsym \
ros-jazzy-moveit-servo \
ros-jazzy-moveit-setup-app-plugins-dbgsym \
ros-jazzy-moveit-setup-app-plugins \
ros-jazzy-moveit-setup-assistant-dbgsym \
ros-jazzy-moveit-setup-assistant \
ros-jazzy-moveit-setup-controllers-dbgsym \
ros-jazzy-moveit-setup-controllers \
ros-jazzy-moveit-setup-core-plugins-dbgsym \
ros-jazzy-moveit-setup-core-plugins \
ros-jazzy-moveit-setup-framework-dbgsym \
ros-jazzy-moveit-setup-framework \
ros-jazzy-moveit-setup-srdf-plugins-dbgsym \
ros-jazzy-moveit-setup-srdf-plugins \
ros-jazzy-moveit-simple-controller-manager-dbgsym \
ros-jazzy-moveit-simple-controller-manager \
ros-jazzy-moveit-visual-tools-dbgsym \
ros-jazzy-moveit-visual-tools \
ros-jazzy-moveit \
ros-jazzy-nav-msgs \
ros-jazzy-object-recognition-msgs \
ros-jazzy-octomap-msgs \
ros-jazzy-ompl \
ros-jazzy-orocos-kdl-vendor \
ros-jazzy-osqp-vendor \
ros-jazzy-osrf-pycommon \
ros-jazzy-parallel-gripper-controller \
ros-jazzy-parameter-traits \
ros-jazzy-pcl-conversions \
ros-jazzy-pcl-msgs \
ros-jazzy-pendulum-control \
ros-jazzy-pendulum-msgs \
ros-jazzy-pid-controller \
ros-jazzy-pilz-industrial-motion-planner \
ros-jazzy-pluginlib \
ros-jazzy-point-cloud-transport \
ros-jazzy-pose-broadcaster \
ros-jazzy-position-controllers \
ros-jazzy-pybind11-vendor \
ros-jazzy-python-cmake-module \
ros-jazzy-python-orocos-kdl-vendor \
ros-jazzy-python-qt-binding \
ros-jazzy-qt-dotgraph \
ros-jazzy-qt-gui-cpp \
ros-jazzy-qt-gui-py-common \
ros-jazzy-qt-gui \
ros-jazzy-quality-of-service-demo-cpp \
ros-jazzy-quality-of-service-demo-py \
ros-jazzy-random-numbers \
ros-jazzy-range-sensor-broadcaster \
ros-jazzy-rcl-action \
ros-jazzy-rcl-interfaces \
ros-jazzy-rcl-lifecycle \
ros-jazzy-rcl-logging-interface \
ros-jazzy-rcl-logging-spdlog \
ros-jazzy-rcl-yaml-param-parser \
ros-jazzy-rcl \
ros-jazzy-rclcpp-action \
ros-jazzy-rclcpp-components \
ros-jazzy-rclcpp-lifecycle \
ros-jazzy-rclcpp \
ros-jazzy-rclpy \
ros-jazzy-rcpputils \
ros-jazzy-rcutils \
ros-jazzy-realtime-tools \
ros-jazzy-resource-retriever \
ros-jazzy-rmw-cyclonedds-cpp \
ros-jazzy-rmw-dds-common \
ros-jazzy-rmw-fastrtps-cpp \
ros-jazzy-rmw-fastrtps-shared-cpp \
ros-jazzy-rmw-implementation-cmake \
ros-jazzy-rmw-implementation \
ros-jazzy-rmw \
ros-jazzy-robot-state-publisher \
ros-jazzy-ros-base \
ros-jazzy-ros-core \
ros-jazzy-ros-environment \
ros-jazzy-ros-gz-bridge-dbgsym \
ros-jazzy-ros-gz-bridge \
ros-jazzy-ros-gz-image-dbgsym \
ros-jazzy-ros-gz-image \
ros-jazzy-ros-gz-interfaces-dbgsym \
ros-jazzy-ros-gz-interfaces \
ros-jazzy-ros-gz-sim-dbgsym \
ros-jazzy-ros-gz-sim-demos \
ros-jazzy-ros-gz-sim \
ros-jazzy-ros-gz \
ros-jazzy-ros-workspace \
ros-jazzy-ros2-control-test-assets \
ros-jazzy-ros2-control \
ros-jazzy-ros2-controllers-test-nodes \
ros-jazzy-ros2-controllers \
ros-jazzy-ros2action \
ros-jazzy-ros2bag \
ros-jazzy-ros2cli-common-extensions \
ros-jazzy-ros2cli \
ros-jazzy-ros2component \
ros-jazzy-ros2controlcli \
ros-jazzy-ros2doctor \
ros-jazzy-ros2interface \
ros-jazzy-ros2launch \
ros-jazzy-ros2lifecycle \
ros-jazzy-ros2multicast \
ros-jazzy-ros2node \
ros-jazzy-ros2param \
ros-jazzy-ros2pkg \
ros-jazzy-ros2run \
ros-jazzy-ros2service \
ros-jazzy-ros2topic \
ros-jazzy-rosbag2-compression-zstd \
ros-jazzy-rosbag2-compression \
ros-jazzy-rosbag2-cpp \
ros-jazzy-rosbag2-interfaces \
ros-jazzy-rosbag2-py \
ros-jazzy-rosbag2-storage-default-plugins \
ros-jazzy-rosbag2-storage-mcap \
ros-jazzy-rosbag2-storage-sqlite3 \
ros-jazzy-rosbag2-storage \
ros-jazzy-rosbag2-transport \
ros-jazzy-rosbag2 \
ros-jazzy-rosgraph-msgs \
ros-jazzy-rosidl-adapter \
ros-jazzy-rosidl-cli \
ros-jazzy-rosidl-cmake \
ros-jazzy-rosidl-core-generators \
ros-jazzy-rosidl-core-runtime \
ros-jazzy-rosidl-default-generators \
ros-jazzy-rosidl-default-runtime \
ros-jazzy-rosidl-dynamic-typesupport-fastrtps \
ros-jazzy-rosidl-dynamic-typesupport \
ros-jazzy-rosidl-generator-c \
ros-jazzy-rosidl-generator-cpp \
ros-jazzy-rosidl-generator-py \
ros-jazzy-rosidl-generator-type-description \
ros-jazzy-rosidl-parser \
ros-jazzy-rosidl-pycommon \
ros-jazzy-rosidl-runtime-c \
ros-jazzy-rosidl-runtime-cpp \
ros-jazzy-rosidl-runtime-py \
ros-jazzy-rosidl-typesupport-c \
ros-jazzy-rosidl-typesupport-cpp \
ros-jazzy-rosidl-typesupport-fastrtps-c \
ros-jazzy-rosidl-typesupport-fastrtps-cpp \
ros-jazzy-rosidl-typesupport-interface \
ros-jazzy-rosidl-typesupport-introspection-c \
ros-jazzy-rosidl-typesupport-introspection-cpp \
ros-jazzy-rpyutils \
ros-jazzy-rqt-action \
ros-jazzy-rqt-bag-plugins \
ros-jazzy-rqt-bag \
ros-jazzy-rqt-common-plugins \
ros-jazzy-rqt-console \
ros-jazzy-rqt-controller-manager \
ros-jazzy-rqt-graph \
ros-jazzy-rqt-gui-cpp \
ros-jazzy-rqt-gui-py \
ros-jazzy-rqt-gui \
ros-jazzy-rqt-image-view \
ros-jazzy-rqt-msg \
ros-jazzy-rqt-plot \
ros-jazzy-rqt-publisher \
ros-jazzy-rqt-py-common \
ros-jazzy-rqt-py-console \
ros-jazzy-rqt-reconfigure \
ros-jazzy-rqt-service-caller \
ros-jazzy-rqt-shell \
ros-jazzy-rqt-srv \
ros-jazzy-rqt-topic \
ros-jazzy-rsl \
ros-jazzy-rttest \
ros-jazzy-ruckig \
ros-jazzy-rviz-assimp-vendor \
ros-jazzy-rviz-common \
ros-jazzy-rviz-default-plugins \
ros-jazzy-rviz-ogre-vendor \
ros-jazzy-rviz-rendering \
ros-jazzy-rviz-visual-tools \
ros-jazzy-rviz2 \
ros-jazzy-sdformat-urdf \
ros-jazzy-sdformat-vendor \
ros-jazzy-sdl2-vendor \
ros-jazzy-sensor-msgs-py \
ros-jazzy-sensor-msgs \
ros-jazzy-service-msgs \
ros-jazzy-shape-msgs \
ros-jazzy-shared-queues-vendor \
ros-jazzy-spdlog-vendor \
ros-jazzy-sqlite3-vendor \
ros-jazzy-srdfdom \
ros-jazzy-sros2-cmake \
ros-jazzy-sros2 \
ros-jazzy-statistics-msgs \
ros-jazzy-std-msgs \
ros-jazzy-std-srvs \
ros-jazzy-steering-controllers-library \
ros-jazzy-stereo-msgs \
ros-jazzy-stomp \
ros-jazzy-tango-icons-vendor \
ros-jazzy-tcb-span \
ros-jazzy-teleop-twist-joy \
ros-jazzy-teleop-twist-keyboard \
ros-jazzy-tf-transformations \
ros-jazzy-tf2-bullet \
ros-jazzy-tf2-eigen-kdl \
ros-jazzy-tf2-eigen \
ros-jazzy-tf2-geometry-msgs \
ros-jazzy-tf2-kdl \
ros-jazzy-tf2-msgs \
ros-jazzy-tf2-py \
ros-jazzy-tf2-ros-py \
ros-jazzy-tf2-ros \
ros-jazzy-tf2-sensor-msgs \
ros-jazzy-tf2-tools \
ros-jazzy-tf2 \
ros-jazzy-theora-image-transport \
ros-jazzy-tinyxml2-vendor \
ros-jazzy-tl-expected \
ros-jazzy-tlsf-cpp \
ros-jazzy-tlsf \
ros-jazzy-topic-monitor \
ros-jazzy-topic-tools-interfaces \
ros-jazzy-topic-tools \
ros-jazzy-tracetools \
ros-jazzy-trajectory-msgs \
ros-jazzy-transmission-interface \
ros-jazzy-tricycle-controller \
ros-jazzy-tricycle-steering-controller \
ros-jazzy-turtlesim \
ros-jazzy-type-description-interfaces \
ros-jazzy-uncrustify-vendor \
ros-jazzy-unique-identifier-msgs \
ros-jazzy-urdf-launch \
ros-jazzy-urdf-parser-plugin \
ros-jazzy-urdf-tutorial \
ros-jazzy-urdf \
ros-jazzy-urdfdom-headers \
ros-jazzy-urdfdom-py \
ros-jazzy-urdfdom \
ros-jazzy-velocity-controllers \
ros-jazzy-vision-msgs \
ros-jazzy-visualization-msgs \
ros-jazzy-warehouse-ros \
ros-jazzy-xacro \
ros-jazzy-yaml-cpp-vendor \
ros-jazzy-zstd-image-transport \
ros-jazzy-zstd-vendor \


# Navigate back to the workspace root
cd /root/ros2_ws

# Install ROS2 dependencies for all packages
#echo "Installing ROS 2 dependencies..."
#sudo rosdep fix-permissions
#rosdep update
#rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

# Now apply all the fixes after dependencies are installed

# Fix storage.cpp
#echo "Fixing storage.cpp..."
#cd /root/ros2_ws/src/moveit_task_constructor
#if [ -f core/src/storage.cpp ]; then
#    # Create backup
#    cp core/src/storage.cpp core/src/storage.cpp.backup
#
#    # Replace the four lines with the new line using sed
#    sed -i '/if (this->end()->scene()->getParent() == this->start()->scene())/,+3c\    this->end()->scene()->getPlanningSceneDiffMsg(t.scene_diff);' core/src/storage.cpp || echo "Warning: Could not modify storage.cpp"
#fi
#
## Fix cartesian_path.cpp
#echo "Fixing cartesian_path.cpp..."
#if [ -f core/src/solvers/cartesian_path.cpp ]; then
#    # Create backup
#    cp core/src/solvers/cartesian_path.cpp core/src/solvers/cartesian_path.cpp.backup
#
#    # Make the replacement
#    sed -i 's/moveit::core::JumpThreshold(props.get<double>("jump_threshold")), is_valid,/moveit::core::JumpThreshold::relative(props.get<double>("jump_threshold")), is_valid,/' core/src/solvers/cartesian_path.cpp || echo "Warning: Could not modify cartesian_path.cpp"
#fi

#cd /root/ros2_ws
#
## Fix PCL warning - this needs to come after rosdep install
#echo "Fixing PCL warnings..."
#find /usr/include/pcl* -path "*/sample_consensus/impl/sac_model_plane.hpp" -exec sed -i 's/^\(\s*\)PCL_ERROR ("\[pcl::SampleConsensusModelPlane::isSampleGood\] Sample points too similar or collinear!\\n");/\1\/\/ PCL_ERROR ("[pcl::SampleConsensusModelPlane::isSampleGood] Sample points too similar or collinear!\\n");/' {} \;
#
## Build the packages
#echo "Building packages..."
## First build without the problematic package
#colcon build --packages-skip mycobot_mtc_pick_place_demo
#source install/setup.bash
#
## Then build the problematic package with warning suppression
#colcon build --packages-select mycobot_mtc_pick_place_demo --cmake-args -Wno-dev
#source install/setup.bash

#echo "Installing Python Dependencies"
#pip install -r /root/requirements.txt

# Final build of everything
colcon build
source install/setup.bash

echo "Workspace setup completed!"
