
import time
import struct
import socket


class SpeedControl:
    def __init__(self, moab_computer, broadcast_address,
                 moab_port=12346, debug_port=27311):

        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(("0.0.0.0", 0))
        self._last_sbus_time = 0
        self._MOAB_COMPUTER = moab_computer
        self._BROADCAST_ADDR = broadcast_address
        self._MOAB_PORT = moab_port
        self._DEBUG_PORT = debug_port
        self._actual_speed = 0
        self._target_speed = 0
        self.K_p = 30
        self.K_i = 20
        self.K_d = 0.0
        self._e = 0.0
        self._I = 0
        self._offset = 0.0
        self._sbus_steering = 1024
        self._sbus_throttle = 1024
        self._DEBUGGING = True
        if self._DEBUGGING:
            self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)


    def set_actual(self, actual_speed):
        # trying to avoid accumulating rounding errors (/noise) when close to 0:
        if actual_speed < 0.05:
            self._actual_speed = 0
            self._I *= 0.99
        else:
            self._actual_speed = actual_speed
        #self.maybe_do_something()

    def set_target(self, steering, target_speed):
        self._sbus_steering = int(round(672 * steering + 1024))
        if self._sbus_steering < 352:
            self._sbus_steering = 352
        elif self._sbus_steering > 1696:
            self._sbus_steering = 1696

        self._target_speed = target_speed
        #self.maybe_do_something()


    def set_throttle(self, steering, throttle):
        self._sbus_steering = int(round(672 * steering + 1024))
        if self._sbus_steering < 352:
            self._sbus_steering = 352
        elif self._sbus_steering > 1696:
            self._sbus_steering = 1696

        self._sbus_throttle = int(round(672 * throttle + 1024))
        if self._sbus_throttle < 1024:
            self._sbus_throttle = 1024
        elif self._sbus_throttle > 1696:
            self._sbus_throttle = 1696

        udpPacket = struct.pack('HH', self._sbus_steering, self._sbus_throttle)
        self.udp_sock.sendto(udpPacket, (self._MOAB_COMPUTER, self._MOAB_PORT))



    def reset_I(self):
        self._I = 0


    def maybe_do_something(self):
        ts = time.time()
        if (ts - self._last_sbus_time) > 0.1:
            self._last_sbus_time = ts

            self._e = self._target_speed - self._actual_speed

            # clip the maximum acceleration:
            if self._e > 1.5:
                self._e = 1.5

            if self._I > 100:
                self._I = 100
            elif self._I < 0:
                self._I = 0

            # braking behaves differently than accelerating, so we have
            # different constants:
            if self._e < 0:
                _K_p = 3.0 * self.K_p
                self._I += 0.333 * self._e
            else:
                _K_p = self.K_p
                self._I += self._e

            output = _K_p * self._e + self.K_i * self._I

            if self._target_speed < -0.9:
                output = -670   # max breaking
                self._I = 0
            elif output < -200:
                output = -200
            elif output > 670:
                output = 670
            self._sbus_throttle = 1024 + int(round(output))


            #print("  ---  TARGET SPEED:  %0.02f" % (self._target_speed))
            udpPacket = struct.pack('HH', self._sbus_steering, self._sbus_throttle)
            self.udp_sock.sendto(udpPacket, (self._MOAB_COMPUTER, self._MOAB_PORT))


            # This is for remote logging, troubleshooting:
            if self._DEBUGGING:
                udpPacket2 = struct.pack('HHfdddddd',
                                 self._sbus_steering, self._sbus_throttle, output,
                                 _K_p, self._e, self.K_i, self._I,
                                 self._target_speed, self._actual_speed)
                self.udp_sock.sendto(udpPacket2, (self._BROADCAST_ADDR, self._DEBUG_PORT))


