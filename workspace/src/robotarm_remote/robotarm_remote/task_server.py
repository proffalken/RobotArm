#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer
from robotarm_msgs.action import RobotarmTask
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

class TaskServer(Node):

    def __init__(self):
        super().__init__("task_server")
        self.get_logger().info("Starting task action server")
        self.action_server = ActionServer(self, RobotarmTask, "task_server", self.goalCallback)
        self.robotarm = MoveItPy(node_name="moveit_py")
        self.robotarm_arm = self.robotarm.get_planning_component("arm")
        self.robotarm_gripper = self.robotarm.get_planning_component("gripper")

    def goalCallback(self, goal_handle):
        self.get_logger().info(f"Received goal request with task_number {goal_handle.request.task_number}")
        
        arm_state = RobotState(self.robotarm.get_robot_model())
        gripper_state = RobotState(self.robotarm.get_robot_model())

        arm_joint_goal = []
        gripper_joint_goal = []

        if goal_handle.request.task_number == 0:
            arm_joint_goal = np.array([0.0, 0.0, 0.0])
            gripper_joint_goal = np.array([-0.7, 0.7])
        elif goal_handle.request.task_number == 1:
            arm_joint_goal = np.array([-1.14, -0.6, -0.7])
            gripper_joint_goal = np.array([0.0, 0.0])
        elif goal_handle.request.task_number == 2:
            arm_joint_goal = np.array([-1.57, 0.0, 0.0])
            gripper_joint_goal = np.array([0.0, 0.0])
        else:
            self.get_logger().error(f"Task Number {goal_handle.request.task_number} was not found")
            return

        arm_state.set_joint_group_positions("arm", arm_joint_goal)
        gripper_state.set_joint_group_positions("gripper", gripper_joint_goal)

        self.robotarm_arm.set_start_state_to_current_state()
        self.robotarm_gripper.set_start_state_to_current_state()

        self.robotarm_arm.set_goal_state(robot_state=arm_state)
        self.robotarm_gripper.set_goal_state(robot_state=gripper_state)

        arm_plan_result = self.robotarm_arm.plan()
        gripper_plan_result = self.robotarm_gripper.plan()

        if arm_plan_result and gripper_plan_result:
            self.robotarm.execute(arm_plan_result.trajectory, controllers=[])
            self.robotarm.execute(gripper_plan_result.trajectory, controllers=[])
        else:
            self.get_logger().error("One or more plans failed")

        goal_handle.succeed()
        result = RobotarmTask.Result()
        result.success = True
        return result



def main():
    rclpy.init()
    task_server = TaskServer()
    rclpy.spin(task_server)
    task_server.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
