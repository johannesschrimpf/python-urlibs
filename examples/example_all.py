#!/usr/bin/env python
"""Example for both tool velocity generator and tool linear generator"""

import time
import sys
import copy

from urlibs.system import URTrajectoryGenerator

__author__ = "Johannes Schrimpf"
__copyright__ = "Copyright 2012, NTNU"
__credits__ = ["Johannes Schrimpf"]
__license__ = "GPL"
__maintainer__ = "Johannes Schrimpf"
__email__ = "johannes.schrimpf(_at_)itk.ntnu.no"
__status__ = "Development"


def example(robot_ip):
    # Initialize the trajectory generator
    robot = URTrajectoryGenerator(robot_ip, log_q_des=True)
    robot.start()
    time.sleep(1)
    if not robot._is_running:
        print("Robot not initialized, exiting...")
        exit()
    # Test tool velocity generator
    robot.start_tvg()
    robot.trajectory_generator._timeout = 1
    robot.trajectory_generator.set_twist([-0.03, 0, 0, 0, 0, 0])
    time.sleep(2)
    # Test tool linear generator
    robot.start_tlg()
    start_pose = robot.get_tool_pose()
    target_pose = copy.copy(start_pose)
    target_pose.pos.x += 0.05
    time.sleep(0.1)
    robot.trajectory_generator.set_target_pose(target_pose)
    time.sleep(1)
    # Test cycle time
    last_time = time.time()
    for dummy in range(10):
        robot.get_tool_pose_blocking()
        print(time.time() - last_time)
        last_time = time.time()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Please give robot ip as argument")
        exit()
    example(sys.argv[1])
