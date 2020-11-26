"""This module provides a joint velocity generator"""
import time
import threading
from trajectory_generator import TrajectoryGenerator
import numpy as np

__author__ = "Johannes Schrimpf"
__copyright__ = "Copyright 2012, NTNU"
__credits__ = ["Johannes Schrimpf"]
__license__ = "GPL"
__maintainer__ = "Johannes Schrimpf"
__email__ = "johannes.schrimpf(_at_)itk.ntnu.no"
__status__ = "Development"


class JointLinearGenerator(TrajectoryGenerator):
    """The joint velocity generator generates a step for the robot
    based on a given joint velocity."""
    def __init__(self, kinematics, cycle_time=0.008, timeout=0.1, q_dot_max=1.5):
        """Initialize the trajectory generator with a timeout.

        Arguments:
        kinematics -- The robot kinematics

        Keyword arguments:
        cycle_time -- The cycle time of the robot interface(default 0.008)
        timeout -- Timeout in s after the last joint velocity was sent to make 
                   sure the robot does not move further when the motion 
                   controller fails. (default 0.1)
        q_dot_max -- Max joint speed (default 1.5)
        """
        TrajectoryGenerator.__init__(self, kinematics, cycle_time=cycle_time)
        self._joint_velocity = np.zeros(kinematics.num_of_joints)
        self._joint_target = None
        
        self._has_target = False
        self._step_count = None
        self._num_steps = None
        self._joint_step = None
        self._last_q = None
        self._init_event = threading.Event()

    def get_step(self, q, qd=None):
        """Returns the next step, based on the given joint velcity 
        
        Arguments:
        q -- The actual joint positions

        Keyword Arguments:
        qd -- The actual joint velocities (default Null)
        """
        if self._last_q is None:
            self._last_q = q
            self._init_event.set()
        else:
            self._last_q = q
        if not self._has_target:
            return q
        
        # Calculate next step
        self._step_count += 1
        if self._step_count <= self._num_steps:
            q = self._last_q + self._joint_step
            # Check whether target is reached
            if self._step_count == self._num_steps:
                # Stop Generator
                self._interpolator = None
                self._has_target = False
                self._step_count = 0
                print("Reached target")
        else:
            #print("Error, this line should not be reached")
            pass
        return q



    def set_joint_target(self, joint_target, velocity=0.5, time_stamp=None):
        """Set the joint target
        
        Arguments:
        joint_target -- The desired joint target

        Keyword arguments:
        time_stamp -- Optional time stamp for debugging and logging 
                      (default None)
        """
        if self._has_target:
            print("JLG has already a target")
            return
        if self._last_q is None:
            # Wait for initialization
            print("Waiting for initialization")
            self._init_event.wait()
        self._joint_velocity = velocity
        self._joint_target = np.array(joint_target)

        distance = self._joint_target - self._last_q
        max_dist = max(abs(distance))
        step_length = self._joint_velocity * self._cycle_time
        self._num_steps = int(np.ceil(max_dist / step_length))
        self._joint_step = distance / self._num_steps
        self._step_count = 0
        if self._num_steps > 0:
            self._has_target = True
        print("num_steps:", self._num_steps)
        print("dist:", distance)
    
    def stop(self):
        """Set zero velocity"""
        for n in range(self._kinematics.num_of_joints):
            self._joint_velocity[n] = 0

    def is_task_idle(self):
        """Returns whether the trajectory controller is task idle"""
        return not self._has_target

    def allows_tool_change(self):
        """Returns whether the trajectory controller allows tool changes"""
        return True

