#!/usr/bin/env python3
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
from enum import Enum

def clamp(val,min,max):
    if val < min:
        val = min
    elif val > max:
        val = max
    return val

class Joints(Enum):
    WAIST = 0
    SHOULDER = 1
    ELBOW = 2
    WRIST = 3
    GRIPPER = 4

# The robot object is what you use to control the robot
robot = InterbotixManipulatorXS("px100", "arm", "gripper")
mode = 'h'

speed = np.deg2rad(20) #deg/s

# Let the user select the position
#https://www.trossenrobotics.com/docs/interbotix_xsarms/specifications/px100.html
while mode != 'q':
    mode=input("[h]ome, [s]leep, [q]uit ")
    if mode == "h":
        robot.arm.go_to_home_pose(moving_time=2)
    elif mode == "s":
        robot.arm.go_to_sleep_pose(moving_time=2)
    elif mode == "go":
        robot.gripper.release(2.0)
    elif mode == "gc":
        robot.gripper.grasp(2.0)
    elif mode.startswith("waist"):
        parse = mode.split(" ")
        if len(parse) < 2:
            continue

        command_position = float(parse[1])
        command_position = clamp(command_position,-179.9,179.9)
        command_position = np.deg2rad(command_position)

        current_position = robot.arm.get_joint_commands()[Joints.WAIST.value]

        time = np.abs(command_position - current_position) / speed

        robot.arm.set_single_joint_position("waist",command_position,time)
    
    elif mode.startswith("sldr"):
        parse = mode.split(" ")
        if len(parse) < 2:
            continue

        command_position = float(parse[1])
        command_position = clamp(command_position,-110.9,106.9)
        command_position = np.deg2rad(command_position)

        current_position = robot.arm.get_joint_commands()[Joints.SHOULDER.value]

        time = np.abs(command_position - current_position) / speed

        robot.arm.set_single_joint_position("shoulder",command_position,time)

    elif mode.startswith("elb"):
        parse = mode.split(" ")
        if len(parse) < 2:
            continue

        command_position = float(parse[1])
        command_position = clamp(command_position,-120.9,91.9)
        command_position = np.deg2rad(command_position)

        current_position = robot.arm.get_joint_commands()[Joints.ELBOW.value]

        time = np.abs(command_position - current_position) / speed

        robot.arm.set_single_joint_position("elbow",command_position,time)

    elif mode.startswith("wrist"):
        parse = mode.split(" ")
        if len(parse) < 2:
            continue

        command_position = float(parse[1])
        command_position = clamp(command_position,-100,100)
        command_position = np.deg2rad(command_position)

        current_position = robot.arm.get_joint_commands()[Joints.WRIST.value]

        time = np.abs(command_position - current_position) / speed

        robot.arm.set_single_joint_position("wrist_angle",command_position,time)