# Robot Arm

**ToDo: Pick a better name**

This is a project to help me understand how robotics work.

There will be many flaws in the design, but I hope to improve those along the way.

## Why this project?

There are a few reasons why I'm doing this:

1. The [servo arm kit](https://www.amazon.co.uk/gp/product/B09TWRWGRL/) I bought to use to learn originally has a huge number of flaws and is basically useless when it comes to repeatability due to issues with servo control and the momentum caused but the weight of the arm moving around
2. I already have various stepper motors and control boards kicking around on my workstation, and I want to reuse them to save money
3. I couldn't tell which of the designs on [Printables](https://www.printables.com) were up to date and supported, so I decided to add to the confusion with my own design
4. I wanted this to be as basic as possible, so I can understand both the mechanics and the code behind basic movement before I move on to more complicated designs

## What will it be capable of?

Eventually, I want the arm to be capable of moving across at least 5 degrees of freedom and change the end-effector (claw etc) by itself. 

There is a long way to go before I get there...

## What can it do now?

At present, there is a digital twin that you can launch with the command `ros2 launch robotarm_bringup simulated_robot.launch.py`.

All the code is based on the excellent [Udemy Robot Manipulator Course](https://www.udemy.com/course/robotics-and-ros-2-learn-by-doing-manipulators/) and a few other things I've picked up from around the web.

I'm working on integrating [grbl_ros](https://github.com/flynneva/grbl_ros) so that the complexity of moving the steppers to the correct location is reduced to sending GCode, but that's not quite working yet.

Once you've built the robot and plugged it all in, you can run the following commands from within the workspace to make it move:

```
### Terminal 1 ###
# Setup the environment
source ./install/setup.bash

# Start the hardware interface
ros2 run grbl_ros grbl_node --ros-args --params-file ./src/grbl_config/config/cnc001.yaml
```

```
### Terminal 2 ###
# Setup the environment
source ./install/setup.bash
# Unlock the CNC controller
ros2 action send_goal /cnc_001/send_gcode_cmd grbl_msgs/action/SendGcodeCmd '{command: $X}'

# Set the unit to mm
ros2 action send_goal /cnc_001/send_gcode_cmd grbl_msgs/action/SendGcodeCmd '{command: G21}'

# Set the distance to relative (so we add/subtract the required distance from the current position)
ros2 action send_goal /cnc_001/send_gcode_cmd grbl_msgs/action/SendGcodeCmd '{command: G91}'

# Send the feed rate and tell it to move the Z axis 15mm counter-clockwise:
ros2 action send_goal /cnc_001/send_gcode_cmd grbl_msgs/action/SendGcodeCmd '{command: G1 F500 Z-15}'

