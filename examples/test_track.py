#!/usr/bin/env python
"""Example for testing the set_tool_transform function"""
import time
import sys
from math3d import Transform

from urlibs.system import TrackURTrajectoryGenerator

__author__ = "Johannes Schrimpf"
__copyright__ = "Copyright 2012, NTNU"
__credits__ = ["Johannes Schrimpf"]
__license__ = "GPL"
__maintainer__ = "Johannes Schrimpf"
__email__ = "johannes.schrimpf(_at_)itk.ntnu.no"
__status__ = "Development"

if __name__ == "__main__":
    if len(sys.argv) == 1:
        robot_ip = "sim"
    else:
        robot_ip = sys.argv[1]
    track_transform = Transform()
    robot = TrackURTrajectoryGenerator(robot_ip, track_transform, None, None)
    robot.start()
    time.sleep(1)
    #t = Transform()
    #t.pos.x = 0.1
    #print robot.get_tool_pose()
    #robot.set_tool_transform(t)
    #print robot.get_tool_pose()
    robot.start_tvg()
