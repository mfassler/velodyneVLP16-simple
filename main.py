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
from misc_map_tools import make_map

from SpeedControl import SpeedControl

MOAB_COMPUTER = "192.168.1.201"
BROADCAST_ADDR = "192.168.1.255"
MOAB_PORT = 12346
SPEED_DEBUG_PORT = 27311

speedControl = SpeedControl(MOAB_COMPUTER, BROADCAST_ADDR, MOAB_PORT, SPEED_DEBUG_PORT)



VELODYNE_PORT = 2368
lidar_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
lidar_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
lidar_sock.bind(("0.0.0.0", VELODYNE_PORT))


# This is lat,lon,heading,speed from "LocationServices":
NAV_PORT = 27201
nav_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
nav_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
nav_sock.bind(("127.0.0.1", NAV_PORT))


LOOKUP_COS = np.empty(36000)
LOOKUP_SIN = np.empty(36000)
for i in range(36000):
    LOOKUP_COS[i] = np.cos(np.radians(i/100.0))
    LOOKUP_SIN[i] = np.sin(np.radians(i/100.0))




WRITE_VIDEO = False
vid_out = None
if len(sys.argv) > 1:
    WRITE_VIDEO = True
    filename = sys.argv[1]
    vid_out = cv2.VideoWriter('cam_video.mjpg', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (800, 800))

DO_NETWORK = False
IMG_RECV_ADDRESS = ('127.0.0.1', 53521)

DO_GUI = True


steering_avg =0.0

class Visualizer:
    def __init__(self):
        height = 800
        width = 800
        self._X_MIN = 2
        self._Y_MIN = 2
        self._X_MAX = (width - 3)
        self._Y_MAX = (height - 3)

        #self.__map = np.ones((height, width, 3), np.uint8) * 255
        self.__map = make_map(800, 800, 50)
        cv2.circle(self.__map, (400, 400), 3, (120,64,0), 2)
        self._map = np.copy(self.__map)
        self._prev_azimuth = 0
        self._count = 0
        self._d_azimuth_history = []


    def draw_a_line(self, rho, theta, color):
        cosTheta = np.cos(theta)
        sinTheta = np.sin(theta)
        x0 = rho * cosTheta
        y0 = rho * sinTheta
        #m = rho * sinTheta / cosTheta
        #b = y0 - m * x0

        x1 = int(x0 - 1000 * sinTheta)
        y1 = int(y0 + 1000 * cosTheta)
        x2 = int(x0 + 1000 * sinTheta)
        y2 = int(y0 - 1000 * cosTheta)

        d_y = 400 - y0
        x_center = 0
        try:
            d_x = (d_y / cosTheta) * sinTheta
            x_center = int(round(x0 - d_x))
        except:
            pass

        cv2.line(self.img,(x1,y1),(x2,y2),color,2)
        return x_center

    def draw_hough_lines(self):
        gray = cv2.bitwise_not(self.img)
        lines = cv2.HoughLines(gray[:, :, 0], 1, np.radians(1), 100)
        #,
        #            min_theta=np.radians(60.0),
        #            max_theta=np.radians(120.0)
        #)

        #usable = False
        left_side = []
        right_side = []
        if lines is not None:
            self.lines = lines
            for line in lines:
                rho, theta = line[0]
                if theta < np.radians(30) or theta > np.radians(150):
                    #print(rho, theta)
                    x_center = self.draw_a_line(rho, theta, (255,0,0))
                    cv2.circle(self.img, (x_center, 400), 3, (0,0,0), 2)

                    if theta > (np.pi / 2.0):
                        theta -= np.pi

                    vec_x = np.sin(theta)
                    vec_y = np.cos(theta)

                    #print("x:", x_center)
                    if x_center < 395:
                        left_side.append( (x_center, vec_x, vec_y))
                    elif x_center > 405:
                        right_side.append( (x_center, vec_x, vec_y))

        _y_center = 400
        if len(left_side) and len(right_side):
            self._left_side = left_side
            self._right_side = right_side

            _l = np.array(left_side)
            _x_center = int(round(_l[:, 0].mean()))
            _xx = int(round(-100 * _l[:, 1].mean()))
            _yy = int(round(-100 * _l[:, 2].mean()))

            x2 = _x_center - _xx
            y2 = _y_center + _yy

            cv2.line(self.img, (_x_center, _y_center), (x2, y2), (0,0,200),3)

            _r = np.array(right_side)
            _x_center = int(round(_r[:, 0].mean()))
            _xx = int(round(-100 * _r[:, 1].mean()))
            _yy = int(round(-100 * _r[:, 2].mean()))

            x2 = _x_center - _xx
            y2 = _y_center + _yy

            cv2.line(self.img, (_x_center, _y_center), (x2, y2), (0,0,200),3)

            _x_center_avg = int(round( 0.5 * (_l[:, 0].mean() + _r[:, 0].mean())))
            avg_x = 0.5 * (_l[:, 1].mean() + _r[:, 1].mean())
            avg_y = 0.5 * (_l[:, 2].mean() + _r[:, 2].mean())

            _xx =  200 * avg_x
            _yy =  200 * avg_y
            #x2 = int(round(avg_x - _xx))
            #y2 = int(round(avg_y - _yy))
            x2 = int(round(_x_center_avg + _xx))
            y2 = int(round(_y_center - _yy))
            x0 = int(round(_x_center_avg - _xx))
            y0 = int(round(_y_center + _yy))

            cv2.line(self.img, (x0, y0), (x2, y2), (0,255,0),2)


            x_target = int(round(_x_center_avg + 100 * avg_x))
            y_target = int(round(_y_center - 100 * avg_y))

            cv2.arrowedLine(self.img, (400, 400), (x_target, y_target), (0,0,0), 2, 8)

            steering = (x_target - 400) / 200.0
            print(steering)
            speedControl.set_throttle(steering, 0.4)


    def show_map(self):
        self.img = self._map
        self._map = np.copy(self.__map)
        self.draw_hough_lines()


        #params = [cv2.IMWRITE_PNG_COMPRESSION, 1]
        params = [cv2.IMWRITE_JPEG_QUALITY, 20]
        if DO_NETWORK:
            _nothing, pngBuffer = cv2.imencode('*.jpg', self.img, params)
            bufLen = len(pngBuffer)
            filepos = 0
            numbytes = 0
            START_MAGIC = b"__HylPnaJY_START_PNG %09d\n" % (bufLen)
            lidar_sock.sendto(START_MAGIC, IMG_RECV_ADDRESS)
            while filepos < bufLen:
                if (bufLen - filepos) < 1400:
                    numbytes = bufLen - filepos
                else:
                    numbytes = 1400  # ethernet MTU is 1500
                lidar_sock.sendto(pngBuffer[filepos:(filepos+numbytes)], IMG_RECV_ADDRESS)
                filepos += numbytes
            STOP_MAGIC = b"_g1nC_EOF"
            lidar_sock.sendto(STOP_MAGIC, IMG_RECV_ADDRESS)

        if DO_GUI:
            cv2.imshow('asdf', self.img)
            cv2.waitKey(1)
        if WRITE_VIDEO:
            vid_out.write(self.img)



    def __not_used__showLines(self):
        lines = cv2.HoughLines(self.img[:, 0], 1, np.pi / 180, 150, None, 0, 0)
        
        if lines is None:
            print('no lines :-(')
            return

        cdst = np.copy(self.img)
        self.lines = lines
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))

            cv2.line(cdst, pt1, pt2, (0,0,255), 2, cv2.LINE_AA)

        cv2.imshow('qwerty', cdst)
        cv2.waitKey(1)


    def parse_data_block(self, data_block):
        flag, azimuth = struct.unpack('HH', data_block[:4])

        # this puts the "seam" in the front
        #if azimuth < self._prev_azimuth:

        # this puts the "seam" in the back (by the cable)
        if self._prev_azimuth < 13500 and azimuth >= 13500:
            myVis.show_map()
            #print(self._count)
            self._count = 0

        d_azimuth = (azimuth - self._prev_azimuth) % 36000
        #self._d_azimuth_history.append(d_azimuth)

        self._prev_azimuth = azimuth

        dist, = struct.unpack('H', data_block[7:9])
        if dist == 0:
            return

        self._count += 1
        r = 0.002 * dist

        x = r * LOOKUP_SIN[azimuth]
        y = r * LOOKUP_COS[azimuth]

        x_px = 400 + int(np.round(x * 50))
        y_px = 400 - int(np.round(y * 50))
        if x_px > self._X_MIN and x_px < self._X_MAX and y_px > self._Y_MIN and y_px < self._Y_MAX:
            self._map[y_px, x_px] = (0,0,0)
            self._map[y_px+1, x_px] = (0,0,0)
            self._map[y_px, x_px+1] = (0,0,0)
            self._map[y_px+1, x_px+1] = (0,0,0)


myVis = Visualizer()


while True:
    inputs, outputs, errors = select.select([lidar_sock, nav_sock], [], [])
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

        elif oneInput == nav_sock:
            #pkt, addr = nav_sock.recvfrom(32)
            pkt, addr = get_last_packet(nav_sock, 32, verbose=False)
            try:
                _current_lat, _current_lon, _current_hdg, _current_spd = \
                    struct.unpack('!dddd', pkt)
            except Exception as ee:
                print('failed to parse location packet:', ee)
            else:
                speedControl.set_actual(_current_spd)







