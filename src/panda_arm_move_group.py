#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import numpy as np
import moveit_commander
from copy import deepcopy
from geometry_msgs.msg import TwistStamped
import geometry_msgs.msg
from math import pi
import copy
import tf

class PandaArmMoveGroup:
    def __init__(self):
        rospy.init_node("panda_arm_move_group", anonymous=False)
        moveit_commander.roscpp_initialize(sys.argv)        # Initialize the move_group API
        self.robot = moveit_commander.RobotCommander()
        self.group_names = self.robot.get_group_names()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('panda_arm')     # Initialize the move group for the ur5_arm
        planning_frame = self.group.get_planning_frame()
        print("============ Reference frame: %s" % planning_frame)
        eef_link = self.group.get_end_effector_link()
        print("============ End effector: %s" % eef_link)
        group_names = self.robot.get_group_names()
        print("============ Robot Groups:", self.robot.get_group_names())
        print("============ Printing robot state")
        print(self.robot.get_current_state())

        # self.end_effector_link = self.group.get_end_effector_link()    # Get the name of the end-effector link
        # self.group.set_pose_reference_frame("/base_link")      # Set the reference frame for pose targets
        # self.group.allow_replanning(True)    # Allow replanning to increase the odds of a solution
        # self.group.set_goal_position_tolerance(0.01)    # Allow some leeway in position (meters) and orientation (radians)
        # self.group.set_goal_orientation_tolerance(0.1)
        # self.group.set_planning_time(0.1)
        # self.group.set_max_acceleration_scaling_factor(.5)
        # self.group.set_max_velocity_scaling_factor(.5)
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/2
        joint_goal[6] = 0
        self.group.set_joint_value_target(joint_goal)
        self.group.set_start_state_to_current_state() # Set the internal state to the current state
        plan = self.group.plan()
        self.group.execute(plan)

        # self.group.go(joint_goal, wait=True)
        # group.stop()


if __name__ == '__main__':
    robot_go = PandaArmMoveGroup()
    # rospy.spin()