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
#   * October 15, 2016 - Encoder and DataLogger classes added
#   *
#   *
#   *
#
##########################################################################################

import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
from bbio.libraries.RotaryEncoder import RotaryEncoder
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
        PWM.set_duty_cycle(self.servo_pin, self.duty_set)

    def close_servo(self):
        PWM.stop(self.servo_pin)
        PWM.cleanup()


class Measurement():
    '''Current measurement in cm and pin info with update methods'''
    def __init__(self, GPIO_pin, measure_type):
        ADC.setup()
        self.GPIO_pin = GPIO_pin
        self.measure_type = measure_type
        self.lookup = {'boom': [[536.0, 564.0, 590.0, 627.0, 667.0, 704.0, 717.0, 741.0, 763.0, 789.0, 812.0, 832.0, 848.0, 864.0, 883.0, 901.0, 914.0],    # BM Analog Input
                                [0, 7.89, 14.32, 22.64, 33.27, 47.4, 50.7, 57.6, 64.2, 72.1, 80.0, 87.4, 93.4, 99.9, 107, 113.4, 118.6]],                   # BM Displacement mm
                       'stick': [[554.0, 602.0, 633.0, 660.0, 680.0, 707.0, 736.0, 762.0, 795.0, 820.0, 835.0, 867.0, 892.0, 919.0, 940.0, 959.0, 983.0, 1007.0, 1019.0],    # SK Analog Input
                                [0, 11.7, 19.2, 26.2, 31, 38.4, 46.3, 53.7, 63.4, 71.5, 76.3, 87.0, 96.4, 106.1, 114.5, 122.1, 132.2, 143.1, 148.5]],                        # SK Displacement mm
                       'bucket': [[173.0, 210.0, 258.0, 297.0, 355.0, 382.0, 429.0, 469.0, 500.0, 539.0, 578.0, 612.0, 628.0, 634.0],
                                [0, 8, 16.5, 23.9, 36.4, 42.4, 53.5, 63.7, 71.6, 81.8, 92.7, 102.5, 107.3, 109]]}

    def update_measurement(self):
        '''In cm'''
        self.value = np.interp(ADC.read_raw(self.GPIO_pin), self.lookup[self.measure_type][0], self.lookup[self.measure_type][1])/10


# class Estimator():  # Obviously unfinished haha
#     '''Temporary class until encoder arrives'''
#     def __init__(self):
#         self.value = 0
#         self.measure_type = 'swing'

#     def update_measurement(self):
#         self.value = 0

class Encoder():
    '''Encoder class to mimic Measurement class and allow listed value calls'''
    def __init__(self):
        self.encoder = RotaryEncoder(RotaryEncoder.EQEP1)
        self.encoder.enable()

        self.encoder.setAbsolute()  # Don't instantiate the class until you want to zero the encoder
        self.encoder.zero()
        self.measure_type = 'swing'

    def update_measurement(self):
        self.value = int(self.encoder.getPosition().split('\n')[0])*np.pi/3200  # Angle in radians, 3200 counts per pi radians, 360 counts per shaft rev, 5 times reduction on shaft, 4 times counts for quadrature


class DataLogger():
    '''Data logging to csv'''
    def __init__(self, mode, filename):
        self.mode = mode
        try:
            self.file = open('data/'+filename, 'w')
        except IOError:
            print('IOError')
        if self.mode == 1:     # Manual mode
            self.file.write('Time,Boom Cmd,Stick Cmd,Bucket Cmd,Swing Cmd,Boom Ms,Stick Ms,Bucket Ms,Swing Ms\n')

        elif self.mode == 2:   # Autonomous mode
            self.file.write('Time,Boom Ms,Stick Ms,Bucket Ms,Swing Ms,Boom Cmd,Stick Cmd,Bucket Cmd,Swing Cmd,Boom Error,Stick Error,Bucket Error,Swing Error\n')

        elif self.mode == 3:   # Blended mode
            self.file.write('Time,Boom Cmd,Stick Cmd,Bucket Cmd,Swing Cmd,Boom Ms,Stick Ms,Bucket Ms,Swing Ms,Boom Ctrl,Stick Ctrl,Bucket Ctrl,Swing Ctrl,Boom Blended,Stick Blended,Bucket Blended,Swing Blended,Primitive,\n')

    def log(self, run_time, data_listed):
        self.file.write(','.join(map(str, [run_time]+data_listed))+'\n')


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
    swing = Servo("P9_42", 4.939, 10, 'Swing')
    actuators = [boom, stick, bucket, swing]

    # Initialize Measurement classes for string pots
    ADC.setup()
    boom_ms = Measurement('P9_37', 'boom')
    stick_ms = Measurement('P9_33', 'stick')
    bucket_ms = Measurement('P9_35', 'bucket')
    swing_ms = Encoder()
    measurements = [boom_ms, stick_ms, bucket_ms, swing_ms]
    return actuators, measurements


def measurement_setup():
    '''Instantiate only the measurement classes'''
    boom_ms = Measurement('P9_37', 'boom')
    stick_ms = Measurement('P9_33', 'stick')
    bucket_ms = Measurement('P9_35', 'bucket')
    swing_ms = Encoder()
    measurements = [boom_ms, stick_ms, bucket_ms, swing_ms]
    return measurements
