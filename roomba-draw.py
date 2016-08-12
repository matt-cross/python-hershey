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
        """
        initialize the robot tracker with robot dimentions (defaults to dimensions for an iRobot roomba)
        r -> the Robot object you want to track.
        counts_per_rev -> how many ticks occur for each encoder when the wheel completes one whole rotation.
        wheel_diam_mm -> what is the diameter or longest distance diagonally across the wheel
        wheel_base_mm -> what is the distance between the two robot wheel (measured from the center of each wheel)
        """
        self.counts_per_rev = counts_per_rev
        self.wheel_diam_mm = wheel_diam_mm
        self.wheel_base_mm = wheel_base_mm

        self.reset(r)

        self.x = 0.0
        self.y = 0.0
        self.theta = math.pi/2 # start out pointing north - toward positiv Y axis
        
    def update(self, r):
        """
        Odometry tracker update. Looks at recent wheel motion and calculates the new
        angle (theta) and position (x,y) of the robot.
        """
        # Get new encoder readings
        new_left = r.sensors['encoder-counts-left']
        new_right = r.sensors['encoder-counts-right']

        # Determine how much the encoder readings have changed (How much rotation of the wheel has occured)
        delta_left = sign_extend_16(new_left - self.prev_left_encoder)
        delta_right = sign_extend_16(new_right - self.prev_right_encoder)

        # Update stored encoder value (Used when this function is called again)
        self.prev_left_encoder = new_left
        self.prev_right_encoder = new_right

        # Convert from encoder ticks to distance the wheel has rolled
        delta_left_mm = delta_left / self.counts_per_rev * (math.pi * 72)
        delta_right_mm = delta_right / self.counts_per_rev * (math.pi * 72)

        # Calculate how the robot as a whole moved both linearly and rotationally
        delta_dist_mm = (delta_left_mm + delta_right_mm) / 2
        delta_angle_rad = (delta_right_mm - delta_left_mm) / self.wheel_base_mm
        
        # Update the total distance travelled and the curent heading
        self.dist_mm += delta_dist_mm
        self.rel_angle_rad += delta_angle_rad

        # Calculate current vector displacement. Note that if the roomba drives in a big circle (not spinning in place) 
        #  the dist_mm value will be equal to the circumference of the circle yet the x and y variables will equal 0.
        #  This is the difference between distance and displacement.
        self.theta += delta_angle_rad
        self.x += math.cos(self.theta) * delta_dist_mm
        self.y += math.sin(self.theta) * delta_dist_mm
        
    def reset(self, r):
        """
        Resets the robot's incremental values for distance travelled and change
         in heading. Does not reset overall robot position and heading variables. 
        """
        self.prev_left_encoder = r.sensors['encoder-counts-left']
        self.prev_right_encoder = r.sensors['encoder-counts-right']

        # Note that the current displacement (x, y) and theta are not reset. 
        self.dist_mm = 0.0
        self.rel_angle_rad = 0.0

    def rel_angle_deg(self):
        """ Returns the robot's change in direction during the last movement in degrees """
        return self.rel_angle_rad * 180 / math.pi

    def rel_angle_rad(self):
        """ Returns the robot's change in direction during the last movement in radians """
        return self.rel_angle_rad

    def rel_distance_mm(self):
        """ Returns the scalar distance travelled during the last movement by the robot in millimeters """
        return self.dist_mm

    def x_mm(self):
        """ Returns the robot's true net displacement only in the vector forward or backwards from its initial direction (Forward being positive) """
        return self.x

    def y_mm(self):
        """ Returns the robot's true net displacement only in the vector perpendicular to its initial direction. (Left being positive) """
        return self.y

    def theta_deg(self):
        """ The true heading of the robot in degrees with CW being positive (Measured with reference to starting position) """
        return self.theta * 180 / math.pi

    def theta_rad(self):
        """ The true heading of the robot in degrees with CW being negative (Measured with reference to starting position) """
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
        """
        Pulls in new information from the create sensors and updates current position.
        """
        self.create.sensors.GetAll()
        self.tracker.update(self.create)

    def tracker_reset(self):
        """
        Resets the distance and angle calculated by the tracker. This should be called at the
        start of each motion that uses rel_angle_deg()/rel_angle_rad() or rel_distance_mm()
        """
        self.tracker.reset(self.create)

def turn_relative(r, new_angle):
    """
    Sends commands to the robot to complete a turn of new_angle degrees. The motor speed sent to the robot 
    is adjusted here to smooth the transitions between moving and being stopped. A negaitve angle will spin
    CW and a positive angle will spin CCW.
    """
    #Resets the value behind rel_angle_deg so that we will turn new_angle relative to our current position
    # rather than relative to our previous position.
    r.tracker_reset()
    
    prev_speed = 0
    max_accel = 5

    # Until we reach our stopping point
    while abs(new_angle - r.tracker.rel_angle_deg()) > 1.0:
        delta_deg = new_angle - r.tracker.rel_angle_deg()
        speed = abs(delta_deg) * 5

        #Gradually change speed to reduce wheel slip and jumpy behavior
        if (speed - prev_speed) > max_accel:
            speed = prev_speed + max_accel

        # We don't want to let speed keep increasing or the decceleration at the end of the turn won't work right
        # We also don't want a velocity below 11 or friction might keep the robot from turning.
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
    angle_stopped = r.tracker.rel_angle_deg()
    r.create.Stop()

    # Measure overshoot:
    time.sleep(0.25) # Arbitrary time to wait for the robot to slow to a stop moving.
    r.tracker_update()

    print ('Attempted to turn {} degrees.  Stopped at {} degrees, then settled at {} degrees'.format(new_angle, angle_stopped, r.tracker.rel_angle_deg()))
    
    
def drive_relative(r, dist_mm):
    #Resets the value behind rel_distance_mm() so that we will drive a distance relative to our current position
    # rather than relative to our previous position.
    r.tracker_reset()
    
    prev_speed = 0
    max_accel = 5
    
    # Until we reach our stopping point
    while abs(dist_mm - r.tracker.rel_distance_mm()) > 1.5:
        delta_mm = dist_mm - r.tracker.rel_distance_mm()
        speed = delta_mm * 3

        #Gradually change speed to avoid wheel slip and jumpy behavior.
        if (speed - prev_speed) > max_accel:
            speed = prev_speed + max_accel
        elif (speed - prev_speed) < -max_accel:
            speed = prev_speed - max_accel

        # We don't want to let speed keep increasing or the decceleration at the end of the movement won't work well
        # We also don't want a velocity below 11 or friction might keep the robot from turning.
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

    # Stop driving:
    dist_stopped = r.tracker.rel_distance_mm()
    r.create.Stop()

    # Measure overshoot:
    time.sleep(0.25) # Arbitrary time to wait for the robot to slow to a stop moving.
    r.tracker_update()

    print ('Attempted to drive {} mm.  Stopped at {} mm, then settled at {} mm'.format(dist_mm, dist_stopped, r.tracker.rel_distance_mm()))
    
    
def turn(r, args):
    """
    Tells the robot to turn the specified number of degrees.
     If the input angle is missing, the robot spins 
     360 degrees in the clockwise direction.
    """
    if len(args) > 0:
        angle = float(args[0])
    else:
        angle = -360

    turn_relative(r, angle)

def drive(r, args):
    """
    Instructs the robot to drive forward a specific number of
    millimeters. If the input distance is missing, the robot
    drives forward for 30mm.
    """
    if len(args) > 0:
        dist_mm = float(args[0])
    else:
        dist_mm = 30

    drive_relative(r, dist_mm)

def goto(r, x, y):
    """
    Tells the robot to go to an absolute position relative to the position and angles the robot started at. Negative values are 
    allowed. 
    x -> How far forward should the robot be compared to where it started
    y -> How far left should the robot be compared to where it started.
    """
    # First figure out our desired heading and distance to target:
    delta_x = x - r.tracker.x_mm()
    delta_y = y - r.tracker.y_mm()
    
    abs_heading_rad = math.atan2(delta_y, delta_x)

    # Make heading relative in degrees and turn to it:
    delta_heading_rad = abs_heading_rad - r.tracker.theta_rad()

    # We should never have to turn further than 180 degrees to face our intended heading. Lets pick
    #  the more efficient way to spin (CW or CCW). 
    while delta_heading_rad > math.pi:
        delta_heading_rad -= math.pi * 2
    while delta_heading_rad < -math.pi:
        delta_heading_rad += math.pi * 2

    turn_relative(r, delta_heading_rad * 180 / math.pi)

    # Recalc distance and move (undershoot slightly on purpose):
    delta_x = x - r.tracker.x_mm()
    delta_y = y - r.tracker.y_mm()
    dist_mm = math.sqrt(delta_x * delta_x + delta_y * delta_y)
    drive_relative(r, dist_mm - 5)
    
def draw(r, args):
    """
    draw(str) allows the user to send string message (Such as \"test\") and the robot will 
     draw the message on the ground. The robot's starting position is the left side of where 
     the message will be printed. The message will be printed in the direction the robot is facing.
    """
    if len(args) < 1:
        text = "Test"
    else:
        text = args[0]

    h = HersheyFont()

    x_scale = 3
    y_scale = -3 # Y axis positive is *down* for Hershey...

    #Raise the pen to start. 
    r.pen.up()
    #We'll work through each letter individually
    for c in list(text):
        #glyph represents how to draw our letter with individual lines. x_origin and y_origin are our starting point the letter will be drawn next to.
        glyph = h.hersheyglyph(c)
        x_origin = r.tracker.x_mm()
        y_origin = r.tracker.y_mm()

        #A glyph contains a series of lines to be drawn that, put together, make a letter. Now we'll draw each line, one by one.
        for line in glyph['lines']:
            first = 1
            #Each line can actually contain multiple points. We will seek out the first point, put down the pen, and move point to point until our line is finished.
            for pt in line:
                if first == 1:
                    first = 0
                    goto(r, x_origin + x_scale * (pt[0] - glyph['left']), y_origin + y_scale * pt[1])
                    r.pen.down()
                else:
                    goto(r, x_origin + x_scale * (pt[0] - glyph['left']), y_origin + y_scale * pt[1])
            #We're done with this line.
            r.pen.up()

            # TODO: Determine if this should be in this (line) loop or the outer (glyph) loop
            # don't forget to move to (right sidebearing, 0) at end of draw
            new_x = x_origin + x_scale * (glyph['right'] - glyph['left'])
            new_y = y_origin
            goto(r, new_x, new_y)

def draw_square(r, args):
    """
    draw a square of user defined size. The first parameter will be used for the dimensions of all four sides. If
     no parameter is given, 30mm sides is assumed. The starting position of the robot is the top left of the square 
     and the robot is facing along what will be the top edge of the square.
    """
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
    """ Raise the pen so that we aren't drawing on the ground """
    r.pen.up()

def pen_down(r, args):
    """ Lower the pen to begin drawing on the ground """
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
