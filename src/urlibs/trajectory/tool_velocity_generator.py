"""This module provides a tool velocity generator"""
import struct
import time
import threading
from PyKDL import Twist 
from trajectory_generator import TrajectoryGenerator
import numpy

__author__ = "Johannes Schrimpf"
__copyright__ = "Copyright 2012, NTNU"
__credits__ = ["Johannes Schrimpf"]
__license__ = "GPL"
__maintainer__ = "Johannes Schrimpf"
__email__ = "johannes.schrimpf(_at_)itk.ntnu.no"
__status__ = "Development"


class ToolVelocityGenerator(TrajectoryGenerator):
    """The tool velocity generator generates a step for the robot
    based on a given twist."""
    def __init__(self, kinematics, cycle_time=0.008, timeout=0.1, interpol=0, q_dot_max=1.5):
        """Initialize the trajectory generator with a timeout.

        Arguments:
        kinematics -- The robot kinematics

        Keyword arguments:
        cycle_time -- The cycle time of the robot interface(default 0.008)
        timeout -- Timeout in s after the last twist was sent to make sure 
                   the robot does not move further when the motion controller 
                   fails. (default 0.1)
        interpol -- Interpolation steps calculation of the next step 
                    (default 1)
        q_dot_max -- Max joint speed (default 1.5)
        """
        TrajectoryGenerator.__init__(self, kinematics, cycle_time=cycle_time)
        self._twist = Twist()
        self._timeout = timeout
        self._twist_update_time = time.time()
        self._log_time_stamp = None
        self._interpol = interpol
        self._q_dot_max = q_dot_max
        self._twist_lock = threading.Lock()
        self._express_frame = None

    def get_step(self, q, qd=None):
        """Returns the next step, based on the given twist
        
        Arguments:
        q -- The actual joint positions

        Keyword Arguments:
        qd -- The actual joint velocities (default Null)
        """
        interpol = self._interpol
        with self._twist_lock:
            twist = self._twist

        if self._express_frame in ("tool", "Tool"):
            temp_twist = numpy.zeros(6)
            for twist_element in range(6):
                temp_twist[twist_element] = self._twist[twist_element] 
            temp_twist = self.get_tool_pose(q).orient * temp_twist
            for twist_element in range(6):
                self._twist[twist_element] = temp_twist[twist_element]

        elif self._express_frame in ("base", "Base"):
            pass
        else:
            print "express_frame '%s' not in ('tool', 'Tool', 'base', 'Base'), interpreting as 'base'" % self._express_frame
            self._express_frame = "base"
        if interpol == 0:
            # No step interpolation
            q_dot = self._kinematics.get_qdot_from_twist(q, twist)
            # Find max joint acceleration
            q_dot_max_act = max(abs(max(q_dot)), abs(min(q_dot)))
            # Scale if too large
            if q_dot_max_act > self._q_dot_max:
                print("Scaling q_dot")
                for joint in range(len(q_dot)):
                    q_dot[joint] *= (self._q_dot_max / q_dot_max_act)
            q_new = q + q_dot * self._cycle_time
        else:
            num_steps = interpol
            # Calculate interpolation time
            delta_t = self._cycle_time / num_steps
            q_new = numpy.array(q)
            for step in range(num_steps):
                q_dot = self._kinematics.get_qdot_from_twist(q_new, twist)
                q_new = q_new + q_dot * delta_t
            # Check max acceleration for calculated step
            q_dot = (q_new - q) / self._cycle_time
            q_dot_max_act = max(abs(max(q_dot)), abs(min(q_dot)))
            # Scale if too large
            if q_dot_max_act > self._q_dot_max:
                print("Scaling q_dot")
                for joint in range(len(q_dot)):
                    q_dot[joint] *= (self._q_dot_max / q_dot_max_act)
                # Replace calculated q_new with scaled q_new
                q_new = q + q_dot * self._cycle_time

        
        # Calculate new joint angles if not timeout.
        if time.time() - self._twist_update_time < self._timeout:
            return q_new
        return q

    def set_twist(self, twist, time_stamp=None, express_frame="Base"):
        """Set the twist
        
        Arguments:
        twist -- The desired twist
        """
        #print "set twist", twist
        self._express_frame = express_frame
        with self._twist_lock:
            for twist_element in range(6):
                self._twist[twist_element] = twist[twist_element]
            self._twist_update_time = time.time()
        self._log_time_stamp = time_stamp
    
    def set_interpol(self, steps):
        """Set number of interpolator steps

        Arguments:
        steps -- number of steps
        """
        self._interpol = steps

    def stop(self):
        """Set zero velocity"""
        for twist_element in range(6):
            self._twist[twist_element] = 0

    def is_task_idle(self):
        """Returns whether the trajectory controller is task idle"""
        return True

    def allows_tool_change(self):
        """Returns whether the trajectory controller allows tool changes"""
        return True

#def performance_test():
#    """Test the performance of the tool velocity generator."""
#    tvg = ToolVelocityGenerator()
#    tvg.set_twist([1, 0, 0, 0, 0, 0])
#    tvg._timeout = 1000
#    q = [0, 0, 0, 0, 0, 0]
#    print tvg.get_step(q)
#    tvg.set_interpol(1)
#    print tvg.get_step(q)
#    tvg.set_interpol(10)
#    print tvg.get_step(q)
#    tvg.set_interpol(100)
#    print tvg.get_step(q)
#    for n in range(3):
#        start_time = time.time()
#        interpol = 10 ** n
#        for m in range(1000):
#            tvg.set_interpol(interpol)
#            tvg.get_step(q)
#        print("interpol=%d: %.5f" % (10 ** n, (time.time() - start_time) / 1000))
#    tvg.set_interpol(1000)
#    print tvg.get_step(q)
#
#if __name__ == "__main__":
#    performance_test()
