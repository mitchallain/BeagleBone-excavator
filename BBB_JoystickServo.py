#! /usr/bin/env python

##########################################################################################
# BBB_JoystickServo.py
#
# Map 1 axis (z-rot) of 3D mouse to PWM control of small servo
#
# NOTE: see evernote for notes on spnav module, and its usage
#
# Created: 07/11/16
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

    servo1 = Servo("P8_34")
    spnav_open()
    try:
        while True:
            event = spnav_poll_event()
            if event is not None:
                angle_input = 180*(-event.rotation[1] + 350)/(700)
                print event.rotation[1]
                servo1.set_servo_angle(angle_input)
    except KeyboardInterrupt:
        print '\nQuitting...'
    finally:
        spnav_close()
        servo1.close_servo()
