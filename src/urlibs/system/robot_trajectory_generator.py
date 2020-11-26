"""This module provides classes to control robots using different
trajectory generators. The setpoints are snet to the robot via a 
connection object."""
from __future__ import print_function

import copy
import time
import sys
import threading
import os
import socket
from urlibs.kinematics import URKinematics, TrackURKinematics
from urlibs.rt import URTCPRouterConnection, RouterDisconnectException, URTCPTrackConnection
from urlibs.trajectory import ToolVelocityGenerator, ZeroVelocityGenerator, \
                              ToolLinearGenerator, JointVelocityGenerator, \
                              JointLinearGenerator

__author__ = "Johannes Schrimpf"
__copyright__ = "Copyright 2012, NTNU"
__credits__ = ["Johannes Schrimpf"]
__license__ = "GPL"
__maintainer__ = "Johannes Schrimpf"
__email__ = "johannes.schrimpf(_at_)itk.ntnu.no"
__status__ = "Development"


class TimeoutEvent():
    def __init__(self, name=""):
        self._event = threading.Event()
        self.name = name
        self._allowed_in_event = threading.Event()
        self._lock = threading.Lock()
        self._n_waiting = 0
        self._allowed_in_event.set()

    def set(self):
        #print("%s: set" % self.name)
        self._allowed_in_event.clear()
        with self._lock:
            if self._n_waiting == 0:
                self._allowed_in_event.set()
            else:
                self._event.set()

    def wait(self, timeout=None):
        #print("%s: wait" % self.name)
        if not self._allowed_in_event.wait(1):
            raise Exception("Client processing lasted more than 1 second")
        with self._lock:
            self._n_waiting += 1
        result = self._event.wait(timeout=timeout)
        #print("%s: LL Result:" % self.name, result)
        with self._lock:
            self._n_waiting -= 1
            if self._n_waiting == 0:
                #print("%s: wait clear ll" % self.name)
                self._event.clear()
                self._allowed_in_event.set()
        return result


class RobotTrajectoryGenerator(threading.Thread):
    """Robot tragectory generator that connects to a robot via agiven robot 
    connection object. It can handle different trajectory generators and 
    initializes with a zero velocity generator."""
    def __init__(self, rob_conn, kinematics, cycle_time=0.008, log_q_des=False, 
                 log_folder=".", log_q_des_name="log_q_des.csv",
                 log_func=print, log_debug_name="", autoconnect=True):
        # TODO: Update documentation
        """Initialize the robot trajectory generator with a given robot 
        connection.

        Arguments:
        rob_conn -- Robot connection object
        kinematics -- The robot kinematics

        Keyword arguments:
        cycle_time -- The cycle time of the robot interface(default 0.008)
        log_q_des -- write log file with commanded joint angles (default False)
        log_q_des_name -- filename of the log (default "log_q_des.csv")
        log_folder -- Name of the logfolder (default ".")
        log_func -- Log function (default print)
        """
        threading.Thread.__init__(self)
        self.daemon = True
        self._rob_conn = rob_conn 
        self._log_q_des = log_q_des
        self._log_folder = log_folder
        self._log_q_des_name = log_q_des_name
        self._logfile_q_des = None
        self._autoconnect = autoconnect

        self._log_debug = False
        self._logfile_debug = None
        if log_debug_name != "":
            self._log_debug = True
            log_name = "urlibs_debug_%s_%.3f.txt" % (log_debug_name, time.time())
            if os.getenv("LOG_PATH") is not None:
                self._log_debug_file = open(os.path.join(os.getenv("LOG_PATH"),
                                                        log_name), 
                                                        "w")
            else:
                self._log_debug_file = open(os.path.join(self._log_folder,
                                                        log_name),
                                                        "w")

        self._q_start = None
        self._q_last_sent = None
        self._gen_name = ""
        self._stop = False
        self._is_running = False
        self._rt_event = TimeoutEvent(name="rt")
        self._q_act = None
        self._qd_act = None
        self._log = log_func
        self._kinematics = kinematics 
        self._init_event = TimeoutEvent()
        self._cycle_time = cycle_time
        self._trajectory_generator = ZeroVelocityGenerator(self._kinematics, cycle_time=cycle_time)

    @property
    def is_running(self):
        """Returns whether the robot thread is running or not"""
        return self._is_running

    def connect(self):
        if not self._rob_conn.is_connected:
            if not self._rob_conn.connect():
                self._log("Could not connect")
        else:
            print("Already connected")

    @property
    def is_connected(self):
        return self._rob_conn.is_connected

    def disconnect(self):
        if self._rob_conn.is_connected:
            self._rob_conn.disconnect()
        else:
            print("Not connected")

    def run(self):
        """Thread main function"""
        self._is_running = True
        # Open logfile
        if self._log_q_des:
            env_log_path = os.getenv("LOG_PATH")
            if env_log_path is not None:
                log_folder = env_log_path 
            else:
                log_folder = self._log_folder
            log_file = os.path.join(log_folder, self._log_q_des_name)
            self._logfile_q_des = open(log_file, "a")

        if self._autoconnect:
            self.connect()

        # Communication loop
        #n = 0
        while not self._stop:
            #n += 1
            if not self._rob_conn.is_connected:
                #self._init_event.clear()
                #if n % 1000 == 0:
                #    print("Not connected, please connect...")
                if self._q_last_sent is not None:
                    self._q_last_sent = None
                if self._q_start is not None:
                    self._q_start = None
                self.start_zvg()
                time.sleep(0.001)
                continue
            try:
                if self._log_debug:
                    self._log_debug_file.write("%.5f get_actual_call\n" %time.time())
                self._q_act, self._qd_act = self._rob_conn.get_actual()
                if self._log_debug:
                    self._log_debug_file.write("%.5f get_actual_resp\n" %time.time())
                self._rt_event.set()
            except socket.error:
                self._log("Could not receive packet: %s" % sys.exc_info()[0] ) 
                raise Exception("Could not receive packet")
            except RouterDisconnectException:
                self._log("Router disconnected")
                self._rob_conn.disconnect()
                continue 
            if self._q_last_sent is None:
                self._q_last_sent = self._q_act
                self._init_event.set()
                time.sleep(0.0001)
            if self._q_start is None:
                self._q_start = self._q_act
            if self._log_debug:
                self._log_debug_file.write("%.5f get_step_call\n" %time.time())
            q_des = self._trajectory_generator.get_step(self._q_last_sent)
            if self._log_debug:
                self._log_debug_file.write("%.5f get_step_resp\n" %time.time())
            self._rob_conn.set_q_desired(q_des)
            if self._log_debug:
                self._log_debug_file.write("%.5f set_q_desired_resp\n" %time.time())
            # Logging
            if self._log_q_des:
                self._logfile_q_des.write(("%.5f, " * 6 + "%.5f\n") % (
                                          time.time(),
                                          q_des[0], q_des[1], q_des[2],
                                          q_des[3], q_des[4], q_des[5]))
            self._q_last_sent = q_des
        # Close logfile
        if self._log_q_des:
            self._logfile_q_des.close()
        if self._log_debug:
            self._log_debug_file.close() 
        self._is_running = False
        self._log("Exit thread")
        #exit(0)

    @property
    def q(self):
        """Return the last commanded q."""
        if self._q_last_sent is None:
            # Wait until first packet from LLC is received
            self._init_event.wait()
        return self._q_last_sent

    @property
    def q_blocking(self):
        """Return the last commanded q."""
        if self._q_last_sent is None:
            # Wait until first packet from LLC is received
            self._init_event.wait()
        self._rt_event.wait()
        return self._q_last_sent

    @property
    def q_act(self):
        """Return the actual q."""
        if self._q_act is None:
            # Wait until first packet from LLC is received
            self._init_event.wait()
        return self._q_act

    def stop(self):
        """Sets the stop variable to stop the thread."""
        self._stop = True

    def start_tvg(self):
        """Sets the ToolVelocityGenerator as trajectory generator."""
        if self._gen_name != "tvg":
            self.wait_for_task_idle()
            self._trajectory_generator = ToolVelocityGenerator(self._kinematics,
                                                               cycle_time=self._cycle_time,
                                                               interpol=0)
            self._gen_name = "tvg"

    def start_zvg(self):
        """Sets the ZeroVelocityGenerator as trajectory generator."""
        if self._gen_name != "zvg":
            self.wait_for_task_idle()
            self._trajectory_generator = ZeroVelocityGenerator(self._kinematics,
                                                               cycle_time=self._cycle_time)
            self._gen_name = "zvg"
        
    def start_tlg(self):
        """Sets the ToolLinearGenerator as trajectory generator."""
        if self._gen_name != "tlg":
            self.wait_for_task_idle()
            self._trajectory_generator = ToolLinearGenerator(self._kinematics,
                                                             cycle_time=self._cycle_time)
            self._gen_name = "tlg"
    
    def start_jvg(self):
        """Sets the JointVelocityGenerator as trajectory generator."""
        if self._gen_name != "jvg":
            self.wait_for_task_idle()
            self._trajectory_generator = JointVelocityGenerator(self._kinematics,
                                                                cycle_time=self._cycle_time)
            self._gen_name = "jvg"
    
    def start_jlg(self):
        """Sets the JointLinearGenerator as trajectory generator."""
        if self._gen_name != "jlg":
            self.wait_for_task_idle()
            self._trajectory_generator = JointLinearGenerator(self._kinematics,
                                                              cycle_time=self._cycle_time)
            self._gen_name = "jlg"
    
    def get_tool_pose(self, timeout=None):
        """Return the tool center frame as vector
        
        Keyword Arguments:
        
        timeout -- The timeout for the init event. Returns None on 
                   timeout (default None)
        """
        q_last_sent = copy.copy(self._q_last_sent) 
        if q_last_sent is None:
            # Wait until first packet from LLC is received
            result = self._init_event.wait(timeout=timeout)
            if not result:
                #Timeout
                return None
            q_last_sent = copy.copy(self._q_last_sent) 
        if q_last_sent is None:
            raise Exception("q_last_sent is None. This should not happen")
        tool_pose = self._trajectory_generator.get_tool_pose(q_last_sent)
        if self.is_connected:
            return tool_pose
        else:
            return None

    def get_flange_pose(self):
        """Return the tool center frame as vector"""
        if self._q_last_sent is None:
            # Wait until first packet from LLC is received
            self._init_event.wait()
        return self._trajectory_generator.get_flange_pose(self._q_last_sent)

    def get_tool_pose_blocking(self, timeout=None):
        """Returns the tool center frame vector (blocking call)
        
        Keyword Arguments:
        
        timeout -- The timeout for the rt event. Returns None on 
                   timeout (default None)
        """
        if self._rt_event.wait(timeout=timeout):
            return self.get_tool_pose(timeout=timeout)
        else:
            return None

    def is_task_idle(self):
        """Returns whether the trajectory controller is task idle"""
        return self._trajectory_generator.is_task_idle()

    def wait_for_task_idle(self):
        """Waits for task idle state of the current controller"""
        while True:
            if self._trajectory_generator.is_task_idle():
                break
            time.sleep(0.002)

    def set_tool_transform(self, tool):
        """Sets a tool to the kinematics chain
        
        Arguments:
        tool -- Segment, Frame or math3d Transform with the tool 
                transformation
        """
        if self._trajectory_generator.allows_tool_change():
            return self._kinematics.set_tool_transform(tool)
        else:
            return False

    def get_tool_transform(self):
        """Returns the tool transformation"""
        return self._kinematics.get_tool_transform()

    @property
    def trajectory_generator(self):
        """Return the current trajectory generator"""
        return self._trajectory_generator


class URTrajectoryGenerator(RobotTrajectoryGenerator):
    """Trajectory generator that initializes a connection to the 
    UR TCP router."""
    def __init__(self, host, cycle_time=0.008, log_q_des=False, 
                 log_q_des_name="log_q_des.csv", log_func=print, log_debug_name=""):
        """Initializes the RobotTragectoryGenerator with a connection to the 
        UR TCP Router
        
        Arguments:
        host -- Host of the UR 

        Keyword arguments:
        cycle_time -- The cycle time of the robot interface(default 0.008)
        log_q_des write log file with commanded joint angles -- (default False)
        log_q_des_name -- filename of the log (defau56lt "log_q_des.csv")
        log_func -- Log function (default print)
        """
        kinematics = URKinematics()
        conn = URTCPRouterConnection(host, 
                                     debug=False, 
                                     log_delay=False, 
                                     log_func=log_func)
        RobotTrajectoryGenerator.__init__(self, 
                                          conn, 
                                          kinematics,
                                          cycle_time=cycle_time,
                                          log_q_des=log_q_des,
                                          log_q_des_name=log_q_des_name,
                                          log_func=log_func,
                                          log_debug_name=log_debug_name)


class TrackURTrajectoryGenerator(RobotTrajectoryGenerator):
    """Trajectory generator that initializes a connection to the 
    UR TCP router."""
    def __init__(self, host, track_transform, lin_axis_getter,
                 lin_axis_setter, cycle_time=0.008, log_q_des=False, 
                 log_q_des_name="log_q_des.csv", log_func=print):
        # TODO: Update documentation
        """Initializes the RobotTragectoryGenerator with a connection to the 
        UR TCP Router
        
        Arguments:
        host -- Host of the UR 
        track_transform -- TODO
        lin_axis_getter -- TODO
        lin_axis_setter -- TODO

        Keyword arguments:
        cycle_time -- The cycle time of the robot interface(default 0.008)
        log_q_des write log file with commanded joint angles -- (default False)
        log_q_des_name -- filename of the log (default "log_q_des.csv")
        log_func -- Log function (default print)
        """
        kinematics = TrackURKinematics(track_transform)
        conn = URTCPTrackConnection(host, 
                                     lin_axis_getter,
                                     lin_axis_setter,
                                     debug=False, 
                                     log_delay=False, 
                                     log_func=log_func)
        RobotTrajectoryGenerator.__init__(self, 
                                          conn, 
                                          kinematics,
                                          cycle_time=cycle_time,
                                          log_q_des=log_q_des,
                                          log_q_des_name=log_q_des_name,
                                          log_func=log_func)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Please give robot ip as argument")
        exit()
    else:
        robot_ip = sys.argv[1]
    # Initialize the trajectory generator
    robot = URTrajectoryGenerator(robot_ip)
    robot.start()
    time.sleep(1)
    if not robot.is_running:
        print("Robot not initialized, exiting...")
        exit()
