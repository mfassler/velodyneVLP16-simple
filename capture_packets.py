#!/usr/bin/env python3
  
import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import time
import pickle
import socket
import select
import argparse

from misc_utils import get_last_packet, sizeof_fmt

parser = argparse.ArgumentParser(description='Capture raw data from a Velodyne VLP-16 Lidar')
parser.add_argument('filename', type=str, help='file to save packets into')

args = parser.parse_args()


VELODYNE_PORT = 2368
lidar_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#lidar_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
lidar_sock.bind(("0.0.0.0", VELODYNE_PORT))

print('Attempting to capture data from a Velodyne VLP-16 Lidar...')

tLast = time.time()
lastSize = 0

f = open(args.filename, 'wb')
while True:
    inputs, outputs, errors = select.select([lidar_sock], [], [], 1.0)
    for oneInput in inputs:
        if oneInput == lidar_sock:
            pkt, addr = get_last_packet(lidar_sock, 2048, verbose=False)
            if len(pkt) == 1206:
                pickle.dump(('vlp16', time.time(), addr, pkt), f, -1)
            else:
                print('Wrong packet size: %d bytes' % (len(pkt)))

    # Send status updates to the screen:
    ts = time.time()
    tDelta = ts - tLast
    if tDelta > 1.0:
        tLast = ts

        speed = (f.tell() - lastSize) / tDelta
        lastSize = f.tell()

        print(sizeof_fmt(f.tell()), '(%s)' % (sizeof_fmt(speed, suffix='B/s')))


