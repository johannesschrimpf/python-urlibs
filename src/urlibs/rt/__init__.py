"""This package provides utilities for accessing the Universal robot via the
TCP Router."""
from tcp_router_connector import URTCPRouterConnection, RouterDisconnectException
from tcp_track_connector import URTCPTrackConnection
from echo_tcp_client import EchoTCPRouterClient

__author__ = "Johannes Schrimpf"
__copyright__ = "Copyright 2012, NTNU"
__credits__ = ["Johannes Schrimpf", "Torstein A. Myhre"]
__license__ = "GPL"
__maintainer__ = "Johannes Schrimpf"
__email__ = "johannes.schrimpf(_at_)itk.ntnu.no"
__status__ = "Development"
