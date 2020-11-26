"""
This module provides a connection object to the TCP router 
running on the Universal Robot control system.
"""
from __future__ import print_function

import numpy as np

__author__ = "Johannes Schrimpf"
__copyright__ = "Copyright 2012, NTNU"
__credits__ = ["Johannes Schrimpf"]
__license__ = "GPL"
__maintainer__ = "Johannes Schrimpf"
__email__ = "johannes.schrimpf(_at_)itk.ntnu.no"
__status__ = "Development"

from tcp_router_connector import URTCPRouterConnection


class LinearConnector(object):
    def __init__(self):
        pass


class LinearAxisEmulator(object):
    def __init__(self):
        self._q = 0.0
        self._qd = 0.0

    def setter(self, q):
        self._qd = q - self._q
        self._q = q

    def getter(self):
        return [self._q, self._qd]


class URTCPTrackConnection(object):
    """
    Low-level connection to the router controlling the Universal Robot.
    A packet must be received and sent to the router in the time it takes 
    to complete one update cycle (0.004ms).
    """
    def __init__(self, host, lin_axis_getter, lin_axis_setter, port=5002, debug=False, log_delay=False, log_func=print):
        """
        Initialize the TCP connection.

        Arguments:
        host -- IP-adress of router.
        lin_axis_getter -- Getter function for the linear axis
        lin_axis_setter -- Setter function for the linear axis

        Keyword Arguments:
        port -- TCP port where router is running. (default 5002)
        debug -- Print debug messages (default False)
        log_delay -- Enable delay logging
        log_func -- Logging funtion

        """
        self._lin_conn = LinearConnector() 
        """
        if host == "sim":
            if lin_axis_getter is not None or lin_axis_setter is not None:
                raise Exception("Getter/Setter given for simulation")
            self._lin_axis_emulator = LinearAxisEmulator()
            self._lin_axis_getter = self._lin_axis_emulator.getter
            self._lin_axis_setter = self._lin_axis_emulator.setter
        """
        if lin_axis_getter is None or lin_axis_setter is None:
            raise Exception("Getter/Setter not valid")
        self._lin_axis_getter = lin_axis_getter
        self._lin_axis_setter = lin_axis_setter
        self._rob_conn = URTCPRouterConnection(host, port=port, debug=debug, log_delay=log_delay, log_func=log_func)
        

    def get_actual(self):
        q_lin, qd_lin = self._lin_axis_getter()
        q_rob, qd_rob = self._rob_conn.get_actual()
        q = np.hstack([q_lin, q_rob]) 
        qd = np.hstack([qd_lin, q_rob])
        return(q, qd)


    def set_q_desired(self, q_desired):
        self._lin_axis_setter(q_desired[0])
        self._rob_conn.set_q_desired(q_desired[1:])

