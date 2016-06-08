#!/usr/bin/python

import pyrobot2
import time

def turn_circle(r):
    # Attempt to put the robot into safe mode
    r.safe = True
    r.Control()

    r.sensors.GetAll()
    if r.sensors['oi-mode'] != 'safe':
        print ('Failed to enter safe mode - is a wheel up or the front over a cliff?')
        return

    # Start turning
    angle_sum = 0
    r.TurnInPlace(300, 'cw')

    # Until we reach our stopping point
    while angle_sum > -360:
	time.sleep(0.1)
        r.sensors.GetAll()
        angle_sum += r.sensors['angle']
	print ('angle_sum now {}'.format(angle_sum))

    # Stop turning:
    r.Stop()

    # Measure overshoot:
    time.sleep(0.5)
    r.sensors.GetAll()
    overshoot_angle = r.sensors['angle']

    print ('Attempted to turn full circle.  Stopped at {} degrees, then overshot {} degrees'.format(angle_sum, overshoot_angle))
    

if __name__ == '__main__':
    r = pyrobot2.Create2('/dev/ttyAMA0')
    turn_circle(r)
