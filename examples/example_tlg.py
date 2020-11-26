#!/usr/bin/env python
"""Example for the tool linear generator"""

import time
import sys
from math3d import Transform

from urlibs.system import URTrajectoryGenerator

__author__ = "Johannes Schrimpf"
__copyright__ = "Copyright 2012, NTNU"
__credits__ = ["Johannes Schrimpf"]
__license__ = "GPL"
__maintainer__ = "Johannes Schrimpf"
__email__ = "johannes.schrimpf(_at_)itk.ntnu.no"
__status__ = "Development"


def example_tlg(robot_ip):
    # Initialize the trajectory generator
    robot = URTrajectoryGenerator(robot_ip, log_q_des=True)
    robot.start()
    time.sleep(1)
    if not robot._is_running:
        print("Robot not initialized, exiting...")
        exit()
    # Test tool linear generator
    robot.start_tlg()
    # Save current pose
    start_pose = robot.get_tool_pose()
    # Create a target pose based on the starting pose
    # Care: target_pose = start_pose would not create
    # a new object, just a pointer to start_pose.
    target_pose = Transform(start_pose)
    target_pose.pos.x += 0.05
    # Wait until tlg is initialized
    time.sleep(0.01)
    # Set target pose
    robot.trajectory_generator.set_target_pose(target_pose)
    # Wait before returning
    time.sleep(1)



if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Please give robot ip as argument")
        exit()
    example_tlg(sys.argv[1])
