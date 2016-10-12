#! /usr/bin/env python

##########################################################################################
# BBB_DualJoystickServo.py
#
# Map 2 axes (z,x-rot) of 3D mouse to PWM control of small servos
#
# NOTE: see evernote for notes on spnav module, 
# for joystick integer ranges, used max = 350, min = -350
#
# Created: July 14, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################


import Adafruit_BBIO.PWM as PWM
from spnav import *


class Servo():

    def __init__(self, servo_pin):
        """Dr. Vaughan's Servo class for creating PWM signals"""
        # Define duty cycle parameters for all servos
        self.duty_min = 3.
        self.duty_max = 14.5
        self.duty_span = self.duty_max - self.duty_min
        self.duty_mid = ((90.0 / 180) * self.duty_span + self.duty_min)

        self.servo_pin = servo_pin
        print 'starting servo PWM'
        PWM.start(self.servo_pin, self.duty_mid, 60.0)

    def set_servo_angle(self, angle):
        angle_f = float(angle)
        duty = ((angle_f / 180) * self.duty_span + self.duty_min)
        PWM.set_duty_cycle(self.servo_pin, duty)

    def close_servo(self):
        PWM.stop(self.servo_pin)
        PWM.cleanup()


if __name__ == "__main__":

    servo1 = Servo("P9_22")
    servo2 = Servo("P8_34")
    spnav_open()
    try:
        while True:
            event = spnav_poll_event()
            if event is not None:
                z_angle_input = 180*(-event.rotation[1] + 350)/(700)
                x_angle_input = 180*(-event.rotation[0] + 350)/(700)
                print event.rotation[0:2]
                servo1.set_servo_angle(z_angle_input)
                servo2.set_servo_angle(x_angle_input)
    except KeyboardInterrupt:
        print '\nQuitting...'
    finally:
        spnav_close()
        servo1.close_servo()
        servo2.close_servo()
