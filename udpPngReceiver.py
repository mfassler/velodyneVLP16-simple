#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")


import os
import select
import socket
import struct
import time
import numpy as np
import cv2 as cv



WRITE_VIDEO = False
vid_out = None

if len(sys.argv) == 2:
    WRITE_VIDEO = True
    vid_filename = sys.argv[1]

if WRITE_VIDEO:
    vid_out = cv.VideoWriter(vid_filename, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'), 20, (2544, 480))


START_MAGIC = b"__HylPnaJY_START_PNG "
STOP_MAGIC = b"_g1nC_EOF"

IMAGE_PORT = 53521


img_socks = [
    socket.socket(socket.AF_INET, socket.SOCK_DGRAM),
]

img_socks[0].bind(("0.0.0.0", IMAGE_PORT))

cv.namedWindow('amap')
cv.moveWindow('amap', 0, 0)


rx_jpgs = {'size': 0, 'packets': [], 'inBand': False}

image = np.zeros((800, 800, 3), np.uint8)


_last_render_time = time.time()
def maybe_render():
    global _last_render_time
    t1 = time.time()
    tDelta = t1 - _last_render_time
    if tDelta > 0.05:
        _last_render_time = t1
        cv.imshow('amap', image)
        cv.waitKey(1)
        if WRITE_VIDEO:
            vid_out.write(frame)


def rx_png_packet(data, addr):
    global rx_jpgs
    global image
    if rx_jpgs['inBand']:
        if data == STOP_MAGIC:
            rx_jpgs['inBand'] = False
            if len(rx_jpgs['packets']) > 1:
                jpgData = b''.join(rx_jpgs['packets'])
                if rx_jpgs['size'] == len(jpgData):
                    rx_jpgs['jpgData'] = np.frombuffer(jpgData, np.uint8)
                    try:
                        im = cv.imdecode(rx_jpgs['jpgData'], cv.IMREAD_UNCHANGED)
                    except Exception as ee:
                        print("Failed to decode jpeg:", ee)
                    else:
                        if im is not None:
                            image = im
                            maybe_render()
                        else:
                            print('im is None')
                else:
                    print('image size doesn\'t match')
        else:
            rx_jpgs['packets'].append(data)
        
    if data.startswith(START_MAGIC):
        rx_jpgs['size'] = int(data[-10:], 10)
        rx_jpgs['packets'] = []
        rx_jpgs['inBand'] = True



while True:
    inputs, outputs, errors = select.select(img_socks, [], [])
    for oneInput in inputs:
        if oneInput == img_socks[0]:
            imgdata, addr = img_socks[0].recvfrom(2048)
            rx_png_packet(imgdata, addr)


