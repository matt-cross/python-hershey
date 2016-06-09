#!/usr/bin/python

import math
import time

import pyrobot2

def sign_extend_16(value):
    return (value & 0x7FFF) - (value & 0x8000)

class RobotKinematics:
    def __init__(self, r, counts_per_rev = 508.8, wheel_diam_mm = 72.0, wheel_base_mm = 235.0):
        self.counts_per_rev = counts_per_rev
        self.wheel_diam_mm = wheel_diam_mm
        self.wheel_base_mm = wheel_base_mm

        self.reset(r)

    def update(self, r):
        new_left = r.sensors['encoder-counts-left']
        new_right = r.sensors['encoder-counts-right']

        delta_left = sign_extend_16(new_left - self.prev_left_encoder)
        delta_right = sign_extend_16(new_right - self.prev_right_encoder)

        self.prev_left_encoder = new_left
        self.prev_right_encoder = new_right

        delta_left_mm = delta_left / self.counts_per_rev * (math.pi * 72)
        delta_right_mm = delta_right / self.counts_per_rev * (math.pi * 72)

        self.distance_mm += (delta_left_mm + delta_right_mm) / 2

        self.angle_rad += (delta_right_mm - delta_left_mm) / self.wheel_base_mm

    def reset(self, r):
        self.prev_left_encoder = r.sensors['encoder-counts-left']
        self.prev_right_encoder = r.sensors['encoder-counts-right']

        self.distance_mm = 0.0
        self.angle_rad = 0.0

    def angle_deg(self):
        return self.angle_rad * 180 / math.pi

    def angle_rad(self):
        return self.angle_rad

    def distance_mm(self):
        return self.distance_mm

def turn_relative(r, k, new_angle):
    k.reset(r)
    
    # Until we reach our stopping point
    while abs(new_angle - k.angle_deg()) > 0.5:
        delta_deg = new_angle - k.angle_deg()
        speed = abs(delta_deg)

        if speed > 200:
            speed = 200
        elif speed < 10:
            speed = 10

        if delta_deg < 0:
            dir = 'cw'
        else:
            dir = 'ccw'

        r.TurnInPlace(speed, dir)
            
        r.sensors.GetAll()
        k.update(r)

    # Stop turning:
    angle_sum = k.angle_deg()
    k.reset(r)
    r.Stop()

    # Measure overshoot:
    time.sleep(0.5)
    r.sensors.GetAll()
    k.update(r)

    print ('Attempted to turn full circle.  Stopped at {} degrees, then settled {} degrees more'.format(angle_sum, k.angle_deg()))
    
    
def turn_circle(r):
    # Attempt to put the robot into safe mode
    r.safe = True
    r.Control()

    r.sensors.GetAll()
    if r.sensors['oi-mode'] != 'safe':
        print ('Failed to enter safe mode - is a wheel up or the front over a cliff?')
        return

    k = RobotKinematics(r)

    turn_relative(r, k, -360)
    
    

if __name__ == '__main__':
    r = pyrobot2.Create2('/dev/ttyAMA0')
    turn_circle(r)
