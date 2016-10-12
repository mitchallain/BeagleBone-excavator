#! /usr/bin/env python

##########################################################################################
# excavator.py
#
# This library provides methods and classes for excavator control
#
# NOTE: created from manual.py and manual_factored.py
#
# Created: October 11, 2016
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
import datetime
import os


class Servo():
    '''Servo class stores pin info and duty limits'''
    def __init__(self, servo_pin, duty_min, duty_max, actuator_name):
        self.duty_min = duty_min
        self.duty_max = duty_max
        self.duty_span = self.duty_max - self.duty_min
        self.duty_mid = ((90.0 / 180) * self.duty_span + self.duty_min)
        self.duty_set = self.duty_mid
        self.actuator_name = actuator_name

        self.servo_pin = servo_pin
        print 'starting servo PWM'
        PWM.start(self.servo_pin, self.duty_mid, 50.625)

    def update_servo(self):
        '''Saturate duty cycle at limits'''
        self.duty_set = max(self.duty_min, min(self.duty_max, self.duty_set))
        PWM.set_duty_cycle(self.duty_set)

    def close_servo(self):
        PWM.stop(self.servo_pin)
        PWM.cleanup()


class Measurement():
    '''Current measurement in cm and pin info with update methods'''
    def __init__(self, GPIO_pin, measure_type):
        ADC.setup()
        self.GPIO_pin = GPIO_pin
        self.measure_type = measure_type
        self.lookup = {'boom': [[536.0, 564.0, 590.0, 627.0, 667.0, 707.0, 753.0, 806.0, 845.0, 872.0, 895.0, 916.0, 923.0],    # BM Analog Input
                                [0, 7.89, 14.32, 22.64, 33.27, 44.05, 56.85, 73.62, 87.54, 98.1, 107.14, 115.34, 118.27]],      # BM Displacement mm
                       'stick': [[558.0, 624.0, 644.0, 685.0, 724.0, 757.0, 786.0, 826.0, 857.0, 891.0, 918.0, 950.0, 977.0, 998.0, 1010.0, 1020.0],    # SK Analog Input
                                [0, 16.28, 21.27, 31.54, 42.06, 51.6, 59.79, 72.94, 83.06, 94.96, 105.03, 117.58, 128.79, 138.52, 143.84, 148]],        # SK Displacement mm
                       'bucket': [[176.0, 263.0, 308.0, 358.0, 417.0, 473.0, 525.0, 576.0, 613.0, 633.0],
                                [0, 17.27, 25.87, 36.6, 50.65, 64.64, 77.93, 91.73, 102.96, 109.24]]}

    def update_measurement(self):
        '''In cm'''
        self.value = np.interp(ADC.read_raw(self.GPIO_pin), self.lookup[self.measure_type][0], self.lookup[self.measure_type][1])/10


class Estimator():  # Obviously unfinished haha
    '''Temporary class until encoder arrives'''
    def __init__(self):
        self.estimation = 0

    def update_measurement(self):
        self.estimation = 0


def parser(received, received_parsed):
    '''Parse joystick data from server_02.py, and convert to float'''
    deadzone = 0.1
    try:
        received = received.translate(None, "[( )]").split(',')
        for axis in range(len(received)):
            if (float(received[axis]) > deadzone) or (float(received[axis]) < -deadzone):
                received_parsed[axis] = float(received[axis])
            else:
                received_parsed[axis] = 0
        return received_parsed
    except ValueError:
        print '\nValue Error'
        raise ValueError


def name_date_time(file_name):
    '''Returns string of format __file__ + '_mmdd_hhmm.csv' '''
    n = datetime.datetime.now()
    data_stamp = os.path.basename(file_name)[:-3] + '_' + n.strftime('%m%d_%H%M')+'.csv'
    return data_stamp


def exc_setup():
    '''Start all PWM classes and measurement classes'''
    boom = Servo("P9_22", 4.939, 10.01, 'Boom')
    stick = Servo("P8_13", 4.929, 8.861, 'Stick')
    bucket = Servo("P8_34", 5.198, 10.03, 'Bucket')
    swing = Servo("P9_28", 4.939, 10, 'Swing')
    actuators = [boom, stick, bucket, swing]

    # Initialize Measurement classes for string pots
    boom_ms = Measurement('P9_37', 'boom')
    stick_ms = Measurement('P9_33', 'stick')
    bucket_ms = Measurement('P9_35', 'bucket')
    swing_est = Estimator()
    measurements = [boom_ms, stick_ms, bucket_ms, swing_est]
    return actuators, measurements
