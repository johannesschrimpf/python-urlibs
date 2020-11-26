"""This class provides a zero velocity trajectory generator."""
from trajectory_generator import TrajectoryGenerator

__author__ = "Johannes Schrimpf"
__copyright__ = "Copyright 2012, NTNU"
__credits__ = ["Johannes Schrimpf"]
__license__ = "GPL"
__maintainer__ = "Johannes Schrimpf"
__email__ = "johannes.schrimpf(_at_)itk.ntnu.no"
__status__ = "Development"

class ZeroVelocityGenerator(TrajectoryGenerator):
    """The Zero Velovity Generator is a trajectory generator that returns
    the actual position as next step."""
    def __init__(self, kinematics, cycle_time=0.008):
        """Initialize the Zero Velocity Generator

        Arguments:
        kinematics -- The robot kinematics
        cycle_time -- The cycle time of the robot interface(default 0.008)
        """
        TrajectoryGenerator.__init__(self, kinematics, cycle_time=cycle_time)

    def get_step(self, q, qd=None):
        """Returns the next step, which is the same joint data as the
        actual joint data.
        
        Arguments:
        q -- The actual joint positions

        Keyword Arguments:
        qd -- The actual joint velocities (default Null)
        """
        return q

    def is_task_idle(self):
        """Returns whether the trajectory controller is task idle"""
        return True

    def allows_tool_change(self):
        """Returns whether the trajectory controller allows tool changes"""
        return True

    def stop(self):
        """Stops the trajectory generator"""
        pass
