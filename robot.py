#!/usr/bin/env python3
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import time
import modern_robotics as mr

joints = {
    "waist" : 0,
    "shoulder" : 1,
    "elbow" : 2,
    "wrist" : 3,
}

class Robot:
    def __init__(self):
        self.ctrl = InterbotixManipulatorXS("px100", "arm", "gripper")
        self.ctrl.gripper.gripper_pressure = 20

    def relMove(self,axis,incr):
        incr = np.deg2rad(incr)

        current_position = self.ctrl.arm.get_joint_commands()[joints[axis]]
        command_position = current_position + incr
        print(f"Current :{current_position}")
        print(f"Command :{command_position}")

        self.ctrl.arm.set_single_joint_position(axis,command_position)

    def absMove(self,axis,pos):
        pos = np.deg2rad(pos)

        self.ctrl.arm.set_single_joint_position(axis,pos)

    def getEndEffector(self):
        joints = self.ctrl.arm.get_joint_commands()
        T = mr.FKinSpace(self.ctrl.arm.robot_des.M, self.ctrl.arm.robot_des.Slist, joints)
        [R,p] = mr.TransToRp(T)

        return p

    def getJointCommand(self,joint):
        return self.ctrl.arm.get_joint_commands()[joints[joint]]

if __name__ == "__main__":
    robot = Robot()

    robot.ctrl.arm.go_to_home_pose()
    robot.getEndEffector()
    robot.relMove("waist", 45)
    robot.getEndEffector()

