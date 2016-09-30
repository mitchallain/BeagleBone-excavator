#! /usr/bin/env python

##########################################################################################
# BBB_JoystickSwing.py
#
# Map 1 axis (z-rot) of 3D mouse to PWM control of swing function
#
# NOTE: duplicated from BBB_JoystickServo.py, except:
#       modified calculation from joystick value to angle, now is input directly to PWM
#       (more straightforward)
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
        self.duty_min = 4.94
        self.duty_max = 9
        self.duty_span = self.duty_max - self.duty_min
        self.duty_mid = ((90.0 / 180) * self.duty_span + self.duty_min)

        self.servo_pin = servo_pin
        print 'starting servo PWM'
        PWM.start(self.servo_pin, self.duty_mid, 50.625)

    def set_servo_angle(self, angle):
        angle_f = float(angle)
        duty = ((angle_f / 180) * self.duty_span + self.duty_min)
        PWM.set_duty_cycle(self.servo_pin, duty)

    def close_servo(self):
        PWM.stop(self.servo_pin)
        PWM.cleanup()


if __name__ == "__main__":

    servo1 = Servo("P9_22")
    spnav_open()
    try:
        while True:
            event = spnav_poll_event()
            if event is not None:
                duty_cycle_input = servo1.duty_span*(-event.rotation[1] + 350)/(700) + servo1.duty_min
                print duty_cycle_input, event.rotation[1]
                PWM.set_duty_cycle(servo1.servo_pin, duty_cycle_input)
    except KeyboardInterrupt:
        print '\nQuitting...'
    finally:
        spnav_close()
        servo1.close_servo()
