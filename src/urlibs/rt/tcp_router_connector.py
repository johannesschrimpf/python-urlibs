"""
This module provides a connection object to the TCP router 
running on the Universal Robot control system.
"""
from __future__ import print_function

import numpy
import socket
import struct
import time
import threading

__author__ = "Torstein Anderssen Myhre, Johannes Schrimpf"
__copyright__ = "Copyright 2012, NTNU"
__credits__ = ["Torstein Anderssen Myhre", "Johannes Schrimpf"]
__license__ = "GPL"
__maintainer__ = "Johannes Schrimpf"
__email__ = "johannes.schrimpf(_at_)itk.ntnu.no"
__status__ = "Development"

class RouterDisconnectException(Exception):
    pass

class Emulator(threading.Thread):
    """RT interface emulation"""
    def __init__(self):
        threading.Thread.__init__(self)
        self.daemon = True
        self._q = numpy.array([-4.0, -1.6, 2.0, -0.3, 1.0, 3.7])
        self._qd = numpy.array([0.0] * 6)
        self._event = threading.Event()
        self._stop = False
        self._is_connected = False 
        self.start()

    def run(self):
        while not self._stop:
            if not self._is_connected:
                time.sleep(0.008)
                continue
            self._event.set()
            self._event.clear()
            time.sleep(0.008)
    
    def connect(self):
        self._is_connected = True

    def disconnect(self):
        self._is_connected = False

    def set_q(self, q_cmd):
        q_cmd = numpy.array(q_cmd)
        self._qd = q_cmd - self._q
        self._q = q_cmd

    def get_actual(self):
        self._event.wait()
        return self._q, self._qd

    def close(self):
        self._stop = True

class URTCPRouterConnection(object):
    """
    Low-level connection to the router controlling the Universal Robot.
    A packet must be received and sent to the router in the time it takes 
    to complete one update cycle (0.004ms).
    """
    def __init__(self, host, port=5002, debug=False, log_delay=False, log_func=print, autoconnect=False):
        """
        Initialize the TCP connection.

        Arguments:
        host -- IP-adress of router.

        Keyword Arguments:
        port -- TCP port where router is running. (default 5002)
        debug -- Print debug messages (default False)
        log_delay -- Enable delay logging (default False)
        log_func -- Logging funtion (default print)
        autoconnect -- Automatically connect when reading packet

        """
        self._q = None
        self._q_dot = None
        self._id = None
        self._cycle_number = None
        self._is_connected = False
        self._host = host
        self._port = port
        self._debug = debug
        self._log_delay = log_delay
        self._recv_time = None
        self._send_time = None
        self._log = log_func
        self._autoconnect = autoconnect
        self._socket_lock = threading.Lock()
        if self._host == "sim":
            self._emulate = True
            self._emulator = Emulator()
        else:
            self._emulate = False
            self._emulator = None

    @property
    def is_connected(self):
        return self._is_connected

    def connect(self):
        """Connect the socket"""
        if self._emulate:
            self._log("Connecting to host: %s, port: %d" % (self._host, self._port))
            self._emulator.connect()
            self._is_connected = True
            return
        with self._socket_lock:
            if not self._is_connected:
                self._log("Connecting to host: %s, port: %d" % (self._host, self._port))
                try:
                    self._sock = socket.create_connection((self._host, self._port))
                    self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                except socket.error:
                    self._log("Error, could not connect to host: %s, port: %d" % (self._host, self._port))
                    self._is_connected = False
                    return False
                    
            self._is_connected = True
        return True
    
    def disconnect(self):
        """Close the tcp connection"""
        self._log("Disconnected from host: %s, port: %d" % (self._host, self._port))
        with self._socket_lock:
            if self._emulator is None:
                self._sock.close()
            else:
                self._emulator.disconnect()
            self._is_connected = False

    def _read_packet(self):
        """Read and parse a packet from the socket."""
        if self._debug:
            self._log("%.6f _read_packet called" % time.time())
        if not self._is_connected:
            if self._autoconnect:
                self.connect()
            else:
                raise Exception("URTCPRouterConnection: Not connected")
        
        with self._socket_lock:
            if self.is_connected:
                raw_packet = self._sock.recv(1024)
                if self._debug:
                    self._log("%f Packet received" % time.time())
                if self._log_delay:
                    self._recv_time = time.time()
                if len(raw_packet) == 101:
                    data = struct.unpack('<cL12d', raw_packet[0:101])
                    self._id = data[0]
                    self._cycle_number = data[1]
                    self._q = numpy.array(data[2:8])
                    self._q_dot = numpy.array(data[8:14])
                elif len(raw_packet) == 0:
                    raise RouterDisconnectException()
                else:
                    raise Exception('Invalid packet length: %d.' % len(raw_packet))
            else:
                raise RouterDisconnectException()

    def get_actual(self):
        """Return the actual joint position and joint velocity"""
        if self._emulate:
            self._q, self._q_dot = self._emulator.get_actual()
        else:
            self._read_packet()
        return (self._q, self._q_dot)
    
    def _print_packet(self):
        """Print packet information"""
        self._log("ProtocolID: %d\tCycle: %d\n" % (ord(self._id), 
                                                   self._cycle_number))
        self._log("q: %s\n" % str(self._q))
        
    
    def _reply_packet(self, q_desired):
        """
        Send packet with joint position setpoint over the socket
        
        Arguments:
        q_desired -- Numpy array of joint angles.
        """
        reply_packet = struct.pack('<cL6d', self._id, 
                                   self._cycle_number, *q_desired)
        with self._socket_lock:
            if self._is_connected:
                self._sock.send(reply_packet)
        if self._debug:
            self._log("%f Packet sent, cycle_number: %d" % (time.time(), self._cycle_number))
        if self._log_delay:
            self._send_time = time.time()
            self._log("Delay: %.5f" % (self._send_time - self._recv_time))

    def set_q_desired(self, q_desired):
        """Send the desired joint values to the robot."""
        if self._emulate:
            self._emulator.set_q(q_desired)
        else:
            self._reply_packet(q_desired)
    
