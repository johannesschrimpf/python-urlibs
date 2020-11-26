#!/usr/bin/env python
from urlibs.kinematics import URKinematics
import math3d as m3d
import numpy as np

__author__ = "Johannes Schrimpf"
__copyright__ = "Copyright 2016, SINTEF Raufoss Manufacturing"
__credits__ = ["Johannes Schrimpf"]
__license__ = "GPL"
__maintainer__ = "Johannes Schrimpf"
__email__ = "johannes.schrimpf(_at_)sintef.no"
__status__ = "Development"

urkin = URKinematics()

q_act = np.array([0, 0, 0, 0, 0, 0])
target_pose = m3d.Transform(m3d.Orientation([0, 0, 0]), m3d.Vector([0.3, 0.3, 0]))

q = urkin.get_ik_from_pose(target_pose, q_act)
print q
