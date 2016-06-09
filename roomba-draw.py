#!/usr/bin/python

import argparse
import math
import sys
import time

import hersheyparse
import hersheymap_parse
import pyrobot2

# Hershey vector font class:
class HersheyFont:
    def __init__(self):
        self.mapfile = "romans.hmp"
        self.fontfile = "hershey-occidental.dat"

        # Parse the font file
        self.glyphs = {}
        for line in open(self.fontfile, 'r'):
            glyph = hersheyparse.hersheyparse(line.rstrip())
            self.glyphs[glyph['charcode']] = glyph

        # Parse the character map file
        self.hersheymap = hersheymap_parse.parse(self.mapfile)

    # Given an ascii character, return the hershey glyph object associated with it.
    def hersheyglyph(self, char):
        return self.glyphs[self.hersheymap[ord(char) - 32]]

def sign_extend_16(value):
    return (value & 0x7FFF) - (value & 0x8000)

class RobotKinematics:
    def __init__(self, r, counts_per_rev = 508.8, wheel_diam_mm = 72.0, wheel_base_mm = 235.0):
        self.counts_per_rev = counts_per_rev
        self.wheel_diam_mm = wheel_diam_mm
        self.wheel_base_mm = wheel_base_mm

        self.reset(r)

        self.x = 0.0
        self.y = 0.0
        self.theta = math.pi/2 # start out pointing north - toward positiv Y axis
        
    def update(self, r):
        new_left = r.sensors['encoder-counts-left']
        new_right = r.sensors['encoder-counts-right']

        delta_left = sign_extend_16(new_left - self.prev_left_encoder)
        delta_right = sign_extend_16(new_right - self.prev_right_encoder)

        self.prev_left_encoder = new_left
        self.prev_right_encoder = new_right

        delta_left_mm = delta_left / self.counts_per_rev * (math.pi * 72)
        delta_right_mm = delta_right / self.counts_per_rev * (math.pi * 72)

        delta_dist_mm = (delta_left_mm + delta_right_mm) / 2
        delta_angle_rad = (delta_right_mm - delta_left_mm) / self.wheel_base_mm
        
        self.dist_mm += delta_dist_mm
        self.angle_rad += delta_angle_rad

        self.theta += delta_angle_rad
        self.x += math.cos(self.theta) * delta_dist_mm
        self.y += math.sin(self.theta) * delta_dist_mm
        
    def reset(self, r):
        self.prev_left_encoder = r.sensors['encoder-counts-left']
        self.prev_right_encoder = r.sensors['encoder-counts-right']

        self.dist_mm = 0.0
        self.angle_rad = 0.0

    def angle_deg(self):
        return self.angle_rad * 180 / math.pi

    def angle_rad(self):
        return self.angle_rad

    def distance_mm(self):
        return self.dist_mm

    def x_mm(self):
        return self.x

    def y_mm(self):
        return self.y

    def theta_deg(self):
        return self.theta * 180 / math.pi

    def theta_rad(self):
        return self.theta
    
class Pen:
    def __init__(self, down_pct=40, up_pct=50):
        self.dev = '/dev/servoblaster'
        self.down_cmd = '0={}%\n'.format(down_pct)
        self.up_cmd = '0={}%\n'.format(up_pct)

    def up(self):
	with open(self.dev, 'wb') as f:
            f.write(self.up_cmd)
        time.sleep(0.5)

    def down(self):
	with open(self.dev, 'wb') as f:
            f.write(self.down_cmd)
        time.sleep(0.5)


class Robot:
    def __init__(self, serial_dev = '/dev/ttyAMA0'):
        self.create = pyrobot2.Create2(serial_dev)
        self.create.safe = True
        self.create.Control()
        self.create.sensors.GetAll()
        if self.create.sensors['oi-mode'] != 'safe':
            raise RuntimeError('Failed to enter safe mode - is a wheel up or the front over a cliff?')
        
        self.tracker = RobotKinematics(self.create)
        self.pen = Pen()

    def tracker_update(self):
        self.create.sensors.GetAll()
        self.tracker.update(self.create)

    def tracker_reset(self):
        self.tracker.reset(self.create)

def turn_relative(r, new_angle):
    r.tracker_reset()
    
    prev_speed = 0
    max_accel = 5

    # Until we reach our stopping point
    while abs(new_angle - r.tracker.angle_deg()) > 1.0:
        delta_deg = new_angle - r.tracker.angle_deg()
        speed = abs(delta_deg) * 5

        if (speed - prev_speed) > max_accel:
            speed = prev_speed + max_accel

        if speed > 200:
            speed = 200
        elif speed < 11:
            speed = 11

        if delta_deg < 0:
            dir = 'cw'
        else:
            dir = 'ccw'

        r.create.TurnInPlace(speed, dir)
        prev_speed = speed
        r.tracker_update()

    # Stop turning:
    angle_stopped = r.tracker.angle_deg()
    r.create.Stop()

    # Measure overshoot:
    time.sleep(0.25)
    r.tracker_update()

    print ('Attempted to turn {} degrees.  Stopped at {} degrees, then settled at {} degrees'.format(new_angle, angle_stopped, r.tracker.angle_deg()))
    
    
def drive_relative(r, dist_mm):
    r.tracker_reset()
    
    prev_speed = 0
    max_accel = 5
    
    # Until we reach our stopping point
    while abs(dist_mm - r.tracker.distance_mm()) > 1.5:
        delta_mm = dist_mm - r.tracker.distance_mm()
        speed = delta_mm * 3

        if (speed - prev_speed) > max_accel:
            speed = prev_speed + max_accel
        elif (speed - prev_speed) < -max_accel:
            speed = prev_speed - max_accel

        if speed > 200:
            speed = 200
        elif speed > 0 and speed < 11:
            speed = 11
        elif speed < 0 and speed > -11:
            speed = -11
        elif speed < -200:
            speed = -200

        r.create.DriveStraight(speed)
        prev_speed = speed
            
        r.tracker_update()

    # Stop turning:
    dist_stopped = r.tracker.distance_mm()
    r.create.Stop()

    # Measure overshoot:
    time.sleep(0.25)
    r.tracker_update()

    print ('Attempted to drive {} mm.  Stopped at {} mm, then settled at {} mm'.format(dist_mm, dist_stopped, r.tracker.distance_mm()))
    
    
def turn(r, args):
    if len(args) > 0:
        angle = float(args[0])
    else:
        angle = -360

    turn_relative(r, angle)

def drive(r, args):
    if len(args) > 0:
        dist_mm = float(args[0])
    else:
        dist_mm = 30

    drive_relative(r, dist_mm)

def goto(r, x, y):
    # First figure out our desired heading and distance to target:
    delta_x = x - r.tracker.x_mm()
    delta_y = y - r.tracker.y_mm()
    
    abs_heading_rad = math.atan2(delta_y, delta_x)
    dist_mm = math.sqrt(delta_x * delta_x + delta_y * delta_y)

    # Make heading relative in degrees and turn to it:
    delta_heading_rad = abs_heading_rad - r.tracker.theta_rad()

    while delta_heading_rad > math.pi:
        delta_heading_rad -= math.pi * 2
    while delta_heading_rad < -math.pi:
        delta_heading_rad += math.pi * 2

    turn_relative(r, delta_heading_rad * 180 / math.pi)

    # Move:
    drive_relative(r, dist_mm)
    
def draw(r, args):
    if len(args) < 1:
        text = "Test"
    else:
        text = args[0]

    h = HersheyFont()

    x_scale = 3
    y_scale = -3 # Y axis positive is *down* for Hershey...

    r.pen.up()
    for c in list(text):
        glyph = h.hersheyglyph(c)
        x_origin = r.tracker.x_mm()
        y_origin = r.tracker.y_mm()

        for line in glyph['lines']:
            first = 1
            for pt in line:
                if first == 1:
                    first = 0
                    goto(r, x_origin + x_scale * (pt[0] - glyph['left']), y_origin + y_scale * pt[1])
                    r.pen.down()
                else:
                    goto(r, x_origin + x_scale * (pt[0] - glyph['left']), y_origin + y_scale * pt[1])

            r.pen.up()

            # don't forget to move to (right sidebearing, 0) at end of draw
            new_x = x_origin + x_scale * (glyph['right'] - glyph['left'])
            new_y = y_origin
            goto(r, new_x, new_y)

def draw_square(r, args):
    if len(args) > 0:
        dist_mm = float(args[0])
    else:
        dist_mm = 30

    print ('starting at ({},{}) theta {}'.format(r.tracker.x_mm(), r.tracker.y_mm(), r.tracker.theta_deg()))

    r.pen.down()

    drive_relative(r, dist_mm)
    print ('now at ({},{}) theta {}'.format(r.tracker.x_mm(), r.tracker.y_mm(), r.tracker.theta_deg()))
    turn_relative(r, -90)
    print ('now at ({},{}) theta {}'.format(r.tracker.x_mm(), r.tracker.y_mm(), r.tracker.theta_deg()))
    drive_relative(r, dist_mm)
    print ('now at ({},{}) theta {}'.format(r.tracker.x_mm(), r.tracker.y_mm(), r.tracker.theta_deg()))
    turn_relative(r, -90)
    print ('now at ({},{}) theta {}'.format(r.tracker.x_mm(), r.tracker.y_mm(), r.tracker.theta_deg()))
    drive_relative(r, dist_mm)
    print ('now at ({},{}) theta {}'.format(r.tracker.x_mm(), r.tracker.y_mm(), r.tracker.theta_deg()))
    turn_relative(r, -90)
    print ('now at ({},{}) theta {}'.format(r.tracker.x_mm(), r.tracker.y_mm(), r.tracker.theta_deg()))
    drive_relative(r, dist_mm)
    print ('now at ({},{}) theta {}'.format(r.tracker.x_mm(), r.tracker.y_mm(), r.tracker.theta_deg()))
    turn_relative(r, -90)
    print ('now at ({},{}) theta {}'.format(r.tracker.x_mm(), r.tracker.y_mm(), r.tracker.theta_deg()))

    r.pen.up()

def pen_up(r, args):
    r.pen.up()

def pen_down(r, args):
    r.pen.down()

commands = {
    'turn': turn,
    'drive': drive,
    'draw': draw,
    'draw_square': draw_square,
    'pen_up': pen_up,
    'pen_down': pen_down,
    }

if __name__ == '__main__':
    r = Robot()

    parser = argparse.ArgumentParser()
    parser.add_argument('command', help='Name of command to run, valid commands are {}'.format(commands.keys()))
    parser.add_argument('args', nargs='*', help='Argument(s) to command')

    args = parser.parse_args()

    commands[args.command](r, args.args)
