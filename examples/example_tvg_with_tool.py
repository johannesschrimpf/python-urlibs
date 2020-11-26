#!/usr/bin/env python
"""Example for the tool velocity generator"""

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


def examble_tvg(robot_ip):
    """This example moved the robot 3 s in -x direction."""
    # Initialize the trajectory generator
    robot = URTrajectoryGenerator(robot_ip)
    robot.start()
    time.sleep(1)
    if not robot._is_running:
        print("Robot not initialized, exiting...")
        exit()
    # Set tool transformation
    tool_transform = Transform()
    tool_transform.pos.z = 0.1
    robot.set_tool_transform(tool_transform)
    print("Tool transform: %s" % robot.get_tool_transform())
    # Start the tool velocity generator
    robot.start_tvg()
    # Set timeout to 3 s
    robot.trajectory_generator._timeout = 3
    # Set a twist to move the robot
    robot.trajectory_generator.set_twist([-0.01, 0, 0, 0, 0, 0])
    # Wait befor returning
    time.sleep(3)



if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Please give robot ip as argument")
        exit()
    examble_tvg(sys.argv[1])
