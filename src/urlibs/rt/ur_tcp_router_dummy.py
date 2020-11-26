#!/usr/bin/python
"""
Emulator for the UR TCP router.
"""

__author__ = "Johannes Schrimpf"
__copyright__ = "SINTEF, NTNU 2012"
__credits__ = ["Johannes Schrimpf", "Morten Lind"]
__license__ = "GPL"
__maintainer__ = "Johannes Schrimpf"
__email__ = "johannes.schrimpf(_at_)itk.ntnu.no"
__status__ = "Development"

import socket
import sys
import struct
import time
import argparse
import gc

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('--host', type=str, default=None)
parser.add_argument('--port', type=int, default=5002)
parser.add_argument('--sample_times', type=int, default=0)
parser.add_argument('--warning_time', type=float, default=0.003)
parser.add_argument('--resp_times_name', type=str, default=None)

args = parser.parse_args()

HOST = args.host               # Symbolic name meaning all available interfaces
PORT = args.port              # Arbitrary non-privileged port

n_samples = args.sample_times
warning_time = args.warning_time
if n_samples > 0:
    resp_times = -1.0 * np.ones(n_samples + 5)

servo_packet_struct = struct.Struct('<BI12d')
servo_data = np.array([0, 0] + [0.0, -np.pi/2, - np.pi / 2, -np.pi / 2, np.pi / 2, 0.0] + 6 * [0.0])
pva_packet_struct = struct.Struct('<BI18d')
pva_data = np.array([0x12, 0] + [0.0, -np.pi/2, - np.pi / 2, -np.pi / 2, np.pi / 2, 0.0] + 12 * [0.0])
p_packet_struct = struct.Struct('<BI6d')
p_data = np.array([0x12, 0] + [0.0, -np.pi/2, - np.pi / 2, -np.pi / 2, np.pi / 2, 0.0])

while True:
    try:
        emu_sock = None
        for res in socket.getaddrinfo(HOST, PORT, socket.AF_UNSPEC,
                                      socket.SOCK_STREAM, 0, socket.AI_PASSIVE):
            af, socktype, proto, canonname, sa = res
            try:
                emu_sock = socket.socket(af, socktype, proto)
                emu_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                emu_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except socket.error as msg:
                emu_sock = None
                continue
            try:
                emu_sock.bind(sa)
                emu_sock.listen(1)
            except socket.error as msg:
                emu_sock.close()
                emu_sock = None
                continue
            break
        if emu_sock is None:
            print('Could not open socket')
            sys.exit(1)
        conn, addr = emu_sock.accept()

        conn.settimeout(1)
        print('Connected by %s' % str(addr))
        start_time = time.time()
        counter = 0
        mark_filled = False
        while True:
            t0 = time.time()
            servo_packet = servo_packet_struct.pack(0x12, counter, *(tuple(pva_data[2:14])))
            gc.disable()
            conn.send(servo_packet)
            send_time = time.time()
            resp_packet = conn.recv(1024)
            response_time = time.time() - send_time
            gc.enable()
            if len(resp_packet) == p_packet_struct.size:
                p_data[:] = p_packet_struct.unpack(resp_packet)
                pva_data[2:8] = p_data[2:8]
            elif len(resp_packet) == pva_packet_struct.size:
                pva_data[:] = pva_packet_struct.unpack(resp_packet)
                p_data[2:8] = pva_data[2:8]
            else:
                print("\n!!!!!! Wrong data length: %d !!!!!!" % len(resp_packet))
            if n_samples > 0:
                resp_times = np.roll(resp_times, 1)
                resp_times[0] = response_time
            else:
                if response_time > warning_time:
                    print("Send and recv packet %.4f"%response_time)
            if response_time > 0.004:
                print("\nWarning: Real-Time part tool:%.4f"%(response_time))
            period_sleep_time = 0.008 - (time.time()-t0)
            if period_sleep_time > 0:
                time.sleep(period_sleep_time)
            counter += 1
            # if n_samples > 0 and counter > n_samples + 5 and not mark_filled:
            #     print('Info: sample array filled.')
            #     mark_filled = True
        conn.close()
    except socket.error as e:
        if n_samples > 0:
            print('Saving response time samples')
            if args.resp_times_name is not None:
                resp_times_name = args.resp_times_name
            else:
                resp_times_name = 'resp_times-%d-%.3f' % (n_samples, time.time())
            np.save(resp_times_name, resp_times[5:])
        print(str(e)) 
        print("================")
        print("Reset Connection")
        print("================")
