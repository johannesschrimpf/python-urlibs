"""This module provides a tool linear generator"""
import time
import numpy as np
import threading
import os

from trajectory_generator import TrajectoryGenerator

from math3d.interpolation import SE3Interpolation
from math3d import Transform

__author__ = "Johannes Schrimpf"
__copyright__ = "Copyright 2012, NTNU"
__credits__ = ["Johannes Schrimpf"]
__license__ = "GPL"
__maintainer__ = "Johannes Schrimpf"
__email__ = "johannes.schrimpf(_at_)itk.ntnu.no"
__status__ = "Development"

class ToolLinearGenerator(TrajectoryGenerator):
    """The tool linear generator generates a step for the robot
    based on a given end frame."""
    def __init__(self, kinematics, cycle_time=0.008, log_cart=False):
        """Initialize the trajectory generator.

        Arguments:
        kinematics -- The robot kinematics
        
        Keyword arguments:
        cycle_time -- The cycle time of the robot interface(default 0.008)
        log_cart -- Log the cartesian position (default False)
        """
        TrajectoryGenerator.__init__(self, kinematics, cycle_time=cycle_time)
        self._log_cart = log_cart
        self._twist_update_time = time.time()
        self._start_pose = None
        self._target_pose = None
        self._interpolator = None
        self._velocity = None
        self._has_target = False
        self._step_count = None
        self._num_steps = None
        self._path_length = None
        self._initializing = False
        self._last_q = None
        self._init_event = threading.Event()
        if self._log_cart:
            if os.getenv("LOG_PATH") is not None:
                self._log_cart_file = open(os.path.join(os.getenv("LOG_PATH"),
                                                        "tlg_cart.csv"), "a")
            else:
                self._log_cart_file = open("tlg_cart.csv", "a")

    def get_step(self, q, qd=None):
        """Returns the next step, based on the given twist
        
        Arguments:
        q -- The actual joint positions

        Keyword Arguments:
        qd -- The actual joint velocities (default Null)
        """
        if self._last_q is None:
            self._last_q = q
            self._init_event.set()
            return q # TODO, just for testing here, not neccessary
        else:
            self._last_q = q
        if not self._has_target:
            return q

        # Calculate next step
        self._step_count += 1
        if self._step_count <= self._num_steps:
            cart = self._interpolator(self._step_count / float(self._num_steps))
            tcf_vec = cart.pose_vector.tolist()
            if self._log_cart:
                self._log_cart_file.write(("%.5f," * 5 + "%.5f\n") % (
                tcf_vec[0], tcf_vec[1], tcf_vec[2],
                tcf_vec[3], tcf_vec[4], tcf_vec[5]))
            q = self._kinematics.get_ik_from_pose(cart, q)
            # Check whether target is reached
            if self._step_count == self._num_steps:
                # Stop Generator
                self._interpolator = None
                self._has_target = False
                self._step_count = 0
                #self._initialized = False
                #print("Reached target")
        else:
            print("Error, this line should not be reached")

        return q

    def _initialize_interpolator(self, q, max_rot_vel=0.5):
        """Initialize the interpolator with a given joint vector.
        
        Arguments:
        q -- Joint vector for the start position. This should be the 
             actual robot position

        Keyword Arguments:
        max_rot_vel -- rotational velocity in rad/s (default 0.1)
        """
        if not self._interpolator:
            # Setup poses
            start_pose = self._kinematics.get_tool_pose(q)
            self._start_pose = start_pose
            print("Start Pose: ", start_pose)
            print("Stop Pose: ", self._target_pose)
            # Setup interpolator
            self._interpolator = SE3Interpolation(self._start_pose, 
                                                  self._target_pose)
            # Calculate number of steps
            path_length = self._target_pose.pos.dist(start_pose.pos)
            rot_length = self._target_pose.orient.ang_dist(start_pose.orient)
            lin_step_length = self._velocity * self._cycle_time
            rot_step_length = max_rot_vel * self._cycle_time
            print("lin_dist: " , path_length, " rot_dist: ", rot_length)
            num_lin_steps = int(np.ceil(path_length / lin_step_length))
            num_rot_steps = int(np.ceil(rot_length / rot_step_length))
            self._num_steps = max(num_lin_steps, num_rot_steps)
            print("lin_steps: " , num_lin_steps, " rot_steps: ", num_rot_steps)
            self._step_count = 0
        else:
            print("Error, interpolator already initialized")

    def set_target_pose(self, pose, velocity=0.05):
        """Set the targetpose
        
        Arguments:
        pose -- target pose
        """
        if self._has_target:
            print("TLG has already a target")
            return
        if self._last_q is None:
            # Wait for initialization
            print("Waiting for initialization")
            self._init_event.wait()
        self._velocity = velocity
        self._target_pose = Transform(pose)
        self._initialize_interpolator(self._last_q)
        self._has_target = True
    

    def has_target(self):
        """Returns whether the tool linear generator has a target"""
        return self._has_target

    def is_task_idle(self):
        """Returns whether the trajectory controller is task idle"""
        return not self._has_target

    def allows_tool_change(self):
        """Returns whether the trajectory controller allows tool changes"""
        return not self._has_target

    def stop(self):
        """Set zero velocity"""
        self._interpolator = None
        self._has_target = False
        self._step_count = 0


#def performance_test():
#    """Test the performance of the inverse kinematics solver and the 
#    interpolator of the tool linear generator"""
#    q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
#    tlg = ToolLinearGenerator()
#    kin = URKinematics()
#    tool_pose = kin.get_tool_pose(q)
#    tool_pose.pos.x += 0.01
#    #
#    # Test Inverse Kinematics
#    #
#    start_time = time.time()
#    for n in range(1000):
#        inv_kin = kin.get_ik_from_pose(pose, q)
#    stop_time = time.time()
#    print("1000 IK Iterations: %.5f" % (stop_time - start_time))
#    print("IK Iteration: %.5f" % ((stop_time - start_time) / 1000))
#    #
#    # Test TLG
#    #
#    tlg.set_target_pose(pose)
#    # First step
#    start_time = time.time()
#    tlg.get_step(q)
#    stop_time = time.time()
#    print("First step: %.5f" % (stop_time - start_time))
#    # Following steps
#    for n in range(10):
#        start_time = time.time()
#        tlg.get_step(q)
#        stop_time = time.time()
#        print("Step: %.5f" % (stop_time - start_time))
#
#if __name__ == "__main__":
#    performance_test()

