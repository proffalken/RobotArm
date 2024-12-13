# The ROS2 workspace

ROS2 has a convention that the workspace is where you keep all of your code, so we'll follow that convention here.

## Installing the requirements

You'll want to create a new virtual-env to install the python requirements for development:

```
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

And you'll need to make sure to follow the [ROS2 Installation Instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

This project assumes you are using [ROS2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/index.html) running on Ubuntu 24.04 or equivalent.  If you're using an alternative distribution then you'll need to work out the equivalent commands!

Once the base packages for Jazzy are installed, make sure you also install the following via `apt install`:

```
python3-colcon-argcomplete
ros-jazzy-ackermann-msgs
ros-jazzy-cv-bridge
ros-jazzy-demo-nodes-cpp
ros-jazzy-demo-nodes-py
ros-jazzy-gz-ros2-control
ros-jazzy-gz-ros2-control-dbgsym
ros-jazzy-gz-ros2-control-demos
ros-jazzy-gz-ros2-control-demos-dbgsym
ros-jazzy-hardware-interface
ros-jazzy-launch-testing-ros
ros-jazzy-octomap-msgs
ros-jazzy-ros-gz-interfaces
ros-jazzy-ros2-control
ros-jazzy-ros2-control-test-assets
ros-jazzy-ros2-controllers
ros-jazzy-ros2-controllers-test-nodes
ros-jazzy-ros2action
ros-jazzy-ros2bag
ros-jazzy-ros2cli
ros-jazzy-ros2cli-common-extensions
ros-jazzy-ros2component
ros-jazzy-ros2controlcli
ros-jazzy-ros2doctor
ros-jazzy-ros2interface
ros-jazzy-ros2launch
ros-jazzy-ros2lifecycle
ros-jazzy-ros2multicast
ros-jazzy-ros2node
ros-jazzy-ros2param
ros-jazzy-ros2pkg
ros-jazzy-ros2run
ros-jazzy-ros2service
ros-jazzy-ros2topic
ros-jazzy-rosbag2-storage
ros-jazzy-sros2
ros-jazzy-sros2-cmake
ros-jazzy-tlsf-cpp
```

## Building / running the software

### The control plane

To build and run the ROS2-based control plane for the robot, run the following commands from this directory:

```
# build
colcon build

### ONLY RUN ONE OF THE FOLLOWING DEPENDING ON WHETHER YOU WANT THE UI OR NOT!

# launch services without the UI
ros2 launch robotarm_launch services.launch.py

# launch services and the UI
ros2 launch robotarm_launch ui.launch.py
```

### The robot firmware

This robot is designed to make use of a standard [GRBL](https://github.com/gnea/grbl) or [GRBHAL](https://github.com/grblHAL) control board as used in CNC machines and 3D printers.

The stepper motors should be wired to the board as follows:

| Motor | CNC Connection Port |
|-------|---------------------|
| Base  | X |
| Arm 1 | Y |
| Arm 2 | Z |
| Effector | A |

The board should be configured to allow `A` to act as a separate axis.

As more modules become available for the end effector, the design will be developed to make use of multiple control boards to take account of additional axes.
