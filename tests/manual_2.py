#! /usr/bin/env python

##########################################################################################
# manual.py
#
# Derived from BBB_tm_Full.py
# Manual operation of the excavator. Begins recording data and timestamps
#
# NOTE:
#
# Created: September 14, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
import numpy as np
import time
from pygame_framebuffer import *
import pygame

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

    def close_servo(self):
        PWM.stop(self.servo_pin)
        PWM.cleanup()


def interpolate(pot_reading, actuator):
    if actuator == 'boom':
        y = [0, 7.89, 14.32, 22.64, 33.27, 44.05, 56.85, 73.62, 87.54, 98.1, 107.14, 115.34, 118.27]
        x = [536.0, 564.0, 590.0, 627.0, 667.0, 707.0, 753.0, 806.0, 845.0, 872.0, 895.0, 916.0, 923.0]
        return np.interp(pot_reading, x, y)
    elif actuator == 'stick':
        y = [0, 16.28, 21.27, 31.54, 42.06, 51.6, 59.79, 72.94, 83.06, 94.96, 105.03, 117.58, 128.79, 138.52, 143.84, 148]
        x = [558.0, 624.0, 644.0, 685.0, 724.0, 757.0, 786.0, 826.0, 857.0, 891.0, 918.0, 950.0, 977.0, 998.0, 1010.0, 1020.0]
        return np.interp(pot_reading, x, y)
    elif actuator == 'bucket':
        y = [0, 10.94, 21.22, 35.11, 49.29, 63.68, 75.85, 87.94, 94.5, 101.71, 103.92, 109.24]
        x = [460.0, 495.0, 532.0, 582.0, 630.0, 672.0, 706.0, 738.0, 755.0, 777.0, 782.0, 797.0]
        return np.interp(pot_reading, x, y)


if __name__ == "__main__":

    # Functions (Servos and PWM BESC)
    boom = Servo("P9_22", 4.939, 10.01)
    arm = Servo("P8_13", 4.929, 8.861)
    bucket = Servo("P8_34", 5.198, 10.03)
    swing = Servo("P9_28", 4.939, 10)

    # Measurement initialization
    ADC.setup()
    
    # Pygame workaround
    # Create an instance of the PyScope class
    scope = pyscope()
    scope.test()
    time.sleep(10)
    
    # Setup joysticks
    pygame.joystick.init()
    JS_right = pygame.joystick.Joystick(0)
    JS_left = pygame.joystick.Joystick(1)
    JS_right.init()
    JS_left.init()


    # try:
    #     f_name = raw_input('Name the output file (end with .csv): ')
    #     f = open('data/'+f_name, 'w')
    # except IOError:
    #     print('IOError')

    # f.write('Time,Boom JS,Stick JS,Bucket JS,Swing JS,Boom Cmd,Stick Cmd,Bucket Cmd,Swing Cmd,Boom Ms,Stick Ms,Bucket Ms,Swing Ms\n')
    # start = time.time()

    try:
        while True:
            
            # Read joysticks into variables
            boom_JS = JS_right.get_axis(0)
            stick_JS = JS_left.get_axis(1)
            bucket_JS = JS_right.get_axis(1)
            swing_JS = JS_left.get_axis(0)

            # New duty cycles
            boom_duty_cycle_input = boom.duty_span*(boom_JS + 1)/(2) + boom.duty_min
            arm_duty_cycle_input = arm.duty_span*(stick_JS + 1)/(2) + arm.duty_min
            bucket_duty_cycle_input = bucket.duty_span*(-bucket_JS + 1)/(2) + bucket.duty_min
            swing_duty_cycle_input = swing.duty_span*(swing_JS + 1)/(2) + swing.duty_min

            # # Data Logging
            # f.write(str(time.time()-start) + ',' + str(boom_JS) + ',' + str(stick_JS) +
            #         ',' + str(bucket_JS) + ',' + str(swing_JS) + ',' + str(boom_duty_cycle_input) +
            #         ',' + str(arm_duty_cycle_input) + ',' + str(bucket_duty_cycle_input) + ',' + str(swing_duty_cycle_input) +
            #         ',' + str(interpolate(ADC.read_raw('P9_37'), 'boom')) + ',' + str(interpolate(ADC.read_raw('P9_33'), 'stick')) +
            #         ',' + str(interpolate(ADC.read_raw('P9_35'), 'bucket')) + '\n')

            # Update PWM
            print boom_duty_cycle_input, arm_duty_cycle_input, bucket_duty_cycle_input, swing_duty_cycle_input
            
            # print boom_duty_cycle_input, swing_duty_cycle_input
            PWM.set_duty_cycle(boom.servo_pin, boom_duty_cycle_input)
            PWM.set_duty_cycle(arm.servo_pin, arm_duty_cycle_input)
            PWM.set_duty_cycle(bucket.servo_pin, bucket_duty_cycle_input)
            PWM.set_duty_cycle(swing.servo_pin, swing_duty_cycle_input)

    except KeyboardInterrupt:
        print '\nQuitting'
    finally:
        print '\nClosing PWM signals...'
        boom.close_servo()
        arm.close_servo()
        bucket.close_servo()
        # f.close()
