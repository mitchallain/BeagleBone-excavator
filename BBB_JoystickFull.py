#! /usr/bin/env python

##########################################################################################
# BBB_JoystickFull.py
#
# Map 3 axes (x, y, z-rot) of 3D mouse to PWM control of boom, arm, and bucket functions
#
# NOTE: duplicated from BBB_JoystickSwing.py, except:
#       modified duty cycle to suit boom function
#
#
# Created: July 14, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   * July 15, 2016, added duty cycles as arguments in servo class init
#       * note modified pwms to ensure neutral signals at duty_mid
#   *
#
##########################################################################################


import Adafruit_BBIO.PWM as PWM
from spnav import *


class Servo():

    def __init__(self, servo_pin, duty_min, duty_max):
        """Dr. Vaughan's Servo class for creating PWM signals"""
        self.duty_min = duty_min
        self.duty_max = duty_max
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

    boom = Servo("P9_22", 4.939, 10.01)
    arm = Servo("P8_13", 4.929, 8.861)
    bucket = Servo("P8_34", 5.198, 10.03)
    PWM.set_duty_cycle(arm.servo_pin, 6.895)
    PWM.set_duty_cycle(bucket.servo_pin, 7.614)

    spnav_open()
    try:
        while True:
            event = spnav_poll_event()
            if event is not None:
                boom_duty_cycle_input = boom.duty_span*(-event.rotation[1] + 350)/(700) + boom.duty_min
                arm_duty_cycle_input = arm.duty_span*(-event.rotation[0] + 350)/(700) + arm.duty_min
                bucket_duty_cycle_input = bucket.duty_span*(-event.rotation[2] + 350)/(700) + bucket.duty_min

                print boom_duty_cycle_input, arm_duty_cycle_input, bucket_duty_cycle_input, event.rotation
                PWM.set_duty_cycle(boom.servo_pin, boom_duty_cycle_input)
                PWM.set_duty_cycle(arm.servo_pin, arm_duty_cycle_input)
                PWM.set_duty_cycle(bucket.servo_pin, bucket_duty_cycle_input)
    except KeyboardInterrupt:
        print '\nQuitting...'
    finally:
        spnav_close()
        boom.close_servo()
        arm.close_servo()
        bucket.close_servo()
