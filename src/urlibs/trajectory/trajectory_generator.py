"""This module provides a generic trajectory generator stub"""
import traceback

__author__ = "Johannes Schrimpf"
__copyright__ = "Copyright 2012, NTNU"
__credits__ = ["Johannes Schrimpf"]
__license__ = "GPL"
__maintainer__ = "Johannes Schrimpf"
__email__ = "johannes.schrimpf(_at_)itk.ntnu.no"
__status__ = "Development"

class TrajectoryGenerator:
    """Abstract Tragectory generator"""
    def __init__(self, kinematics, cycle_time=0.008):
        """Initialize the trajectory generator
        
        Arguments:
        kinematics -- The robot kinematics

        Keyword arguments:
        cycle_time -- The cycle time of the robot interface(default 0.008)
        """
        self._kinematics = kinematics 
        self._cycle_time = cycle_time

    def get_tool_pose(self, q):
        """Returns the tool center frame for a given joint configuration.

        Arguments:
        q -- joint configuration
        """
        return self._kinematics.get_tool_pose(q)

    def get_flange_pose(self, q):
        """Returns the flange_pose for a given joint configuration.

        Arguments:
        q -- joint configuration
        """
        return self._kinematics.get_flange_pose(q)

    def get_step(self, q, qd):
        """Returns the next step, which is the same joint data as the
        actual joint data.
        
        Arguments:
        q -- The actual joint positions

        Keyword Arguments:
        qd -- The actual joint velocities (default Null)
        """
        raise NotImplementedError(
          '"' + traceback.extract_stack(limit=1)[-1][2] +'"'
          +' not implemented in abstract base class "TrajectoryGenerator"')

    def is_task_idle(self):
        """Returns whether the trajectory controller is task idle"""
        raise NotImplementedError(
          '"' + traceback.extract_stack(limit=1)[-1][2] +'"'
          +' not implemented in abstract base class "TrajectoryGenerator"')
        

    def stop(self):
        """Stops the trajectory generator"""
        raise NotImplementedError(
          '"' + traceback.extract_stack(limit=1)[-1][2] +'"'
          +' not implemented in abstract base class "TrajectoryGenerator"')
    
    def allows_tool_change(self):
        """Returns whether the trajectory controller allows tool changes"""
        raise NotImplementedError(
          '"' + traceback.extract_stack(limit=1)[-1][2] +'"'
          +' not implemented in abstract base class "TrajectoryGenerator"')

    
