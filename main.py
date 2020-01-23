#!/usr/bin/env python3
  
import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import os
import time
import select
import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
import cv2

from misc_utils import get_last_packet


VELODYNE_PORT = 2368
lidar_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
lidar_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
lidar_sock.bind(("0.0.0.0", VELODYNE_PORT))



LOOKUP_COS = np.empty(36000)
LOOKUP_SIN = np.empty(36000)
for i in range(36000):
    LOOKUP_COS[i] = np.cos(np.radians(i/100.0))
    LOOKUP_SIN[i] = np.sin(np.radians(i/100.0))



class Visualizer:
    def __init__(self):
        height = 800
        width = 1400
        self._X_MIN = 2
        self._Y_MIN = 2
        self._X_MAX = (width - 3)
        self._Y_MAX = (height - 3)

        self.__map = np.ones((height, width, 3), np.uint8) * 255
        self._map = np.copy(self.__map)
        self._prev_azimuth = 0
        self._count = 0
        #self._d_azimuth_history = []

    def show_map(self):
        img = self._map
        self._map = np.copy(self.__map)
        cv2.imshow('asdf', img)
        cv2.waitKey(1)

    def parse_data_block(self, data_block):
        flag, azimuth = struct.unpack('HH', data_block[:4])

        # this puts the "seam" in the front
        #if azimuth < self._prev_azimuth:

        # this puts the "seam" in the back (by the cable)
        if self._prev_azimuth < 18000 and azimuth >= 18000:
            myVis.show_map()
            #print(self._count)
            self._count = 0

        #    d_azimuth = azimuth - self._prev_azimuth + 36000
        #else:
        #    d_azimuth = azimuth - self._prev_azimuth

        #self._d_azimuth_history.append(d_azimuth)

        self._prev_azimuth = azimuth

        dist, = struct.unpack('H', data_block[7:9])
        if dist == 0:
            return

        self._count += 1
        r = 0.002 * dist

        x = r * LOOKUP_SIN[azimuth]
        y = r * LOOKUP_COS[azimuth]

        x_px = 700 + int(np.round(x * 50))
        y_px = 400 - int(np.round(y * 50))
        if x_px > self._X_MIN and x_px < self._X_MAX and y_px > self._Y_MIN and y_px < self._Y_MAX:
            self._map[y_px, x_px] = (0,0,0)
            self._map[y_px+1, x_px] = (0,0,0)
            self._map[y_px, x_px+1] = (0,0,0)
            self._map[y_px+1, x_px+1] = (0,0,0)


myVis = Visualizer()


while True:
    inputs, outputs, errors = select.select([lidar_sock], [], [])
    for oneInput in inputs:
        if oneInput == lidar_sock:
            #pkt, addr = lidar_sock.recvfrom(2048)
            pkt, addr = get_last_packet(lidar_sock, 2048, verbose=False)

            if len(pkt) == 1206:
                ts, fac = struct.unpack('IH', pkt[-6:])
                for i in range(12):
                    iStart = i*100
                    iStop = iStart + 100
                    data_block = pkt[iStart:iStop]
                    myVis.parse_data_block(data_block)



