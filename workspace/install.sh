#!/bin/bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_GB en_GB.UTF-8
sudo update-locale LC_ALL=en_GB.UTF-8 LANG=en_GB.UTF-8
export LANG=en_GB.UTF-8

locale  # verify settings


sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt install -y ros-dev-tools

sudo apt install -y python3-colcon-argcomplete \
ros-jazzy-ackermann-msgs \
ros-jazzy-cv-bridge \
ros-jazzy-demo-nodes-cpp \
ros-jazzy-demo-nodes-py \
ros-jazzy-gz-ros2-control \
ros-jazzy-gz-ros2-control-dbgsym \
ros-jazzy-gz-ros2-control-demos \
ros-jazzy-gz-ros2-control-demos-dbgsym \
ros-jazzy-hardware-interface \
ros-jazzy-launch-testing-ros \
ros-jazzy-octomap-msgs \
ros-jazzy-ros-gz-interfaces \
ros-jazzy-ros2-control \
ros-jazzy-ros2-control-test-assets \
ros-jazzy-ros2-controllers \
ros-jazzy-ros2-controllers-test-nodes \
ros-jazzy-ros2action \
ros-jazzy-ros2bag \
ros-jazzy-ros2cli \
ros-jazzy-ros2cli-common-extensions \
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
ros-jazzy-rosbag2-storage \
ros-jazzy-sros2 \
ros-jazzy-sros2-cmake \
ros-jazzy-tlsf-cpp 
