"""This module provides an echo client for the tcp router."""
import sys
import threading
from tcp_router_connector import URTCPRouterConnection

__author__ = "Johannes Schrimpf"
__copyright__ = "Copyright 2012, NTNU"
__credits__ = ["Johannes Schrimpf"]
__license__ = "GPL"
__maintainer__ = "Johannes Schrimpf"
__email__ = "johannes.schrimpf(_at_)itk.ntnu.no"
__status__ = "Development"

class EchoTCPRouterClient(threading.Thread):
    """Echo client for the TCP Router. It stores the first received position
    and commands this position to thr robot until stopped"""
    def __init__(self, host):
        """Initializes the connection"""
        threading.Thread.__init__(self)
        self.daemon = True
        self._conn = URTCPRouterConnection(host)
        self._stop = False
        self._q_start = None
    
    def run(self):
        """The main function of the thread"""
        while not self._stop:
            q, qd = self._conn.get_actual()
            if self._q_start is None:
                self._q_start = q
            self._conn.set_q_desired(self._q_start)
        self._conn.disconnect()

    def stop(self):
        """Set the stop variable and wait for thread to stop"""
        self._stop = True
        self.join()
        


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python echo_tcp_client.py ip-address")
        exit(0)
    echo_client = EchoTCPRouterClient(sys.argv[1])
    echo_client.start()
    raw_input("Press Enter to stop")
    echo_client.stop()

