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


class JointVelocityGenerator(TrajectoryGenerator):
    """The joint velocity generator generates a step for the robot
    based on a given joint velocity."""
    def __init__(self, kinematics, cycle_time=0.008, timeout=0.1, q_dot_max=1.5, q_dotdot_max=10.0):
        """Initialize the trajectory generator with a timeout.

        Arguments:
        kinematics -- The robot kinematics

        Keyword arguments:
        cycle_time -- The cycle time of the robot interface(default 0.008)
        timeout -- Timeout in s after the last joint velocity was sent to make 
                   sure the robot does not move further when the motion 
                   controller fails. (default 0.1)
        q_dot_max -- Max joint speed (default 1.5)
        q_dotdot_max -- Max joint acceleration (default 10.0)
        """
        TrajectoryGenerator.__init__(self, kinematics, cycle_time=cycle_time)
        self._joint_velocity = np.zeros(kinematics.num_of_joints)
        self._timeout = timeout
        self._joint_velocity_update_time = time.time()
        self._log_time_stamp = None
        self._q_dot_last = np.zeros(self._kinematics.num_of_joints)
        self._q_dot_max = q_dot_max
        self._q_dotdot_max = q_dotdot_max
        self._lock = threading.Lock()

    def get_step(self, q, qd=None):
        """Returns the next step, based on the given joint velcity 
        
        Arguments:
        q -- The actual joint positions

        Keyword Arguments:
        qd -- The actual joint velocities (default Null)
        """
        # Stop on timeout.
        with self._lock:
            if time.time() - self._joint_velocity_update_time > self._timeout:
                self._joint_velocity = np.zeros(self._kinematics.num_of_joints)
            q_dot_target = self._joint_velocity

        # Check max joint acceleration 
        q_dot_delta = q_dot_target - self._q_dot_last
        
        step_max = self._q_dotdot_max * self._cycle_time


        q_dot_delta_abs_max = np.max(np.abs(q_dot_delta))
        if q_dot_delta_abs_max > step_max:
            factor = float(step_max) / q_dot_delta_abs_max
        else:
            factor = 1
        q_dot = self._q_dot_last + q_dot_delta * factor
        #print "T:", q_dot_target[5], "A:", self._q_dot_last[5], "f:", factor, "step_max", step_max, "maxabs", q_dot_delta_abs_max

        # Check max joint velocity 
        q_dot_max_act = np.max(np.abs(q_dot))
        # Scale if too large
        if q_dot_max_act > self._q_dot_max:
            print("Scaling q_dot")
            for joint in range(len(q_dot)):
                q_dot[joint] *= (self._q_dot_max / q_dot_max_act)
        q_new = q + q_dot * self._cycle_time

        self._q_dot_last = q_dot
        return q_new


    def set_joint_velocity(self, joint_velocity, time_stamp=None):
        """Set the joint velocity
        
        Arguments:
        joint_velocity -- The desired joint velocity

        Keyword arguments:
        time_stamp -- Optional time stamp for debugging and logging 
                      (default None)
        """
        with self._lock:
            for n in range(self._kinematics.num_of_joints):
                self._joint_velocity[n] = joint_velocity[n]
            self._joint_velocity_update_time = time.time()
        self._log_time_stamp = time_stamp
    
    def stop(self):
        """Set zero velocity"""
        for n in range(self._kinematics.num_of_joints):
            self._joint_velocity[n] = 0

    def is_task_idle(self):
        """Returns whether the trajectory controller is task idle"""
        return True

    def allows_tool_change(self):
        """Returns whether the trajectory controller allows tool changes"""
        return True

