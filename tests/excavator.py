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
#   * October 16, 2016 - js_index and toggle_invert added to Servo class to sort out inputs, probably a better way
#   * October 19, 2016 - Added trigger based prediction class, TriggerPrediction, and reordered parser output
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
    '''A generic PWM class

    Args:
        servo_pin (str): String representing BeagleBone pin, ex: 'P9_32'
        duty_min (float): Minimum duty cycle
        duty_max (float): Maximum duty cycle
        actuator_name (str, optional): Used identify actuators
        js_index (int, optional): Index of joystick list corresponding to this actuator

    Attributes:
        duty_min (float): Minimum duty cycle
        duty_max (float): Maximum duty cycle
        duty_span (float): Difference between duty_max and duty_min
        duty_mid (float): Average of duty_max and duty_min
        actuator_name (str, optional): Used identify actuators
        js_index (int, optional): Index of joystick list corresponding to this actuator
        '''
    def __init__(self, servo_pin, duty_min, duty_max, actuator_name='', js_index=0):
        self.duty_min = duty_min
        self.duty_max = duty_max
        self.duty_span = self.duty_max - self.duty_min
        self.duty_mid = ((90.0 / 180) * self.duty_span + self.duty_min)
        self.duty_set = self.duty_mid
        self.actuator_name = actuator_name
        self.js_index = js_index

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
    '''Current measurement in cm and pin info with update methods

    Args:
        GPIO_pin (str): String representation of BeagleBone Black pin, ex. "P9_32"
        measure_type (str): String name of measurement type, must match lookup tables below

    Attributes:
        GPIO_pin (str): see above
        measure_type (str): see above
        lookup (dict): dictionary with lookup tables for each string potentiometer, must be calibrated regularly
        value (float): displacement of actuator in cm
    '''
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
        '''Uses lookup tables and current analog in value to find actuator displacement'''
        self.value = np.interp(ADC.read_raw(self.GPIO_pin), self.lookup[self.measure_type][0], self.lookup[self.measure_type][1])/10


# class Estimator():  # Obviously unfinished haha
#     '''Temporary class until encoder arrives'''
#     def __init__(self):
#         self.value = 0
#         self.measure_type = 'swing'

#     def update_measurement(self):
#         self.value = 0

class Encoder():
    '''Encoder class to mimic Measurement class and allow listed value calls

    Attributes:
        encoder (obj): instance of RotaryEncoder class for EQEP1
        measure_type (str): String representing the type of measurement
        value (float): position of encoder in radians
    '''
    def __init__(self):
        self.encoder = RotaryEncoder(RotaryEncoder.EQEP1)
        self.encoder.enable()

        self.encoder.setAbsolute()  # Don't instantiate the class until you want to zero the encoder
        self.encoder.zero()
        self.measure_type = 'swing'

    def update_measurement(self):
        '''Function which updates measurement value for encoder'''
        self.value = int(self.encoder.getPosition().split('\n')[0])*np.pi/3200  # Angle in radians, 3200 counts per pi radians, 360 counts per shaft rev, 5 times reduction on shaft, 4 times counts for quadrature


class DataLogger():
    '''Data logging to csv

    Args:
        mode (int): 1 for manual, 2 for autonomous, 3 for Blended
        filename (str): string to write values to, ending in ".csv"

    Attributes:
        mode (int): see above
        file (obj): file object for writing data
    '''
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

        elif self.mode == 3:   # Blended mode (Commands, Controllers, Blended, Measurements, Class, Probability)
            self.file.write('Time,Boom Cmd,Stick Cmd,Bucket Cmd,Swing Cmd,Boom Ctrl,Stick Ctrl,Bucket Ctrl,Swing Ctrl,Boom Blended,Stick Blended,Bucket Blended,Swing Blended,Boom Ms,Stick Ms,Bucket Ms,Swing Ms,Class,Confidence,\n')

    def log(self, data_listed):
        self.file.write(','.join(map(str, data_listed))+'\n')


class Prediction():
    '''Prediction class does all the magic.

    Args:
        mode (int): 0 is off, 1 is static alpha, 2 is dynamic alpha
        model (string): string referencing the active task model
        alpha (float): BSC blending parameter preset for static mode

    Attributes:
        mode (int): see above
        model (obj):
        endpoints (list, floats): endpoints for the current task
        confidence (float): probability that current task is nominal
        blend_threshold (float): mininum confidence to initiate blending
        alpha (float): BSC blending parameter alpha
        primitive: current blending primitive
        history: primitives and endpoints from recent history (window TBD)
    '''
    def __init__(self, mode, model='', alpha=0):
        self.mode = mode
        if self.mode == 0:  # Blending off
            self.alpha = 0
        elif self.mode == 1:  # Static alpha
            self.alpha = alpha
        # elif self.mode == 2:
            #  We will see what goes here

    def update_prediction(self, js_inputs, measurements):
        '''Update our predictions for motion class and endpoints'''
        # This is where all the magic happens


# DEPRECATED
# # Trying out some subgoal dicts, since they do not need to have information storage and are essentially just task models
# sg_pile = {'subgoal': 1,
#            'it': [3, -0.5],                                  # Joystick index 3 (swing) move left
#            'subgoal_pos': [6.7528, 0.9117, 9.9466, 1.4058],  # Over the pile
#            'npt': [3, 3, 3, 0.2],                            # Terminate when swing close to the pile
#            'onpt': []}

# sg_dump = {'subgoal': 2,                                     # WILL BE 6, JUST NEED THE ITERATOR TO WORK
#            'subgoal_pos': [6.7528, 0.9117, 9.9466, 0.32],    # Subgoal position: uncurled over truck
#            'it': [0, 0, -0.5, 0],                               # Input Trigger: bucket uncurl cmd
#            'npt': [3, 3, 3, 0.2],                            # NPT: bucket uncurled
#            'onpt': []}                                       # ONPT:

# sg_dig_model = [sg_pile, sg_dump]           # Listed dicts form model
# del sg_pile, sg_dump                        # Clean-up namespace


class TriggerPrediction():
    '''The trigger prediction class uses task specific event triggers to determine the current subgoal

    Args:
        mode (int): 0 is off, 1 is static alpha, 2 is dynamic alpha
        model (list: dicts): list of subgoal model dicts, see example below
        alpha (float): BSC blending parameter preset for static mode

    Example arg:
        sg_model = [{'subgoal': 1,
                     'it': [3, -0.5]                            * Joystick index 3 (swing) move past halfway left
                     'subgoal_pos': [6.75, 0.91, 9.95, 1.41]    * Over the pile (actuator space coordinates)
                     'npt': [3, 3, 3, 0.2]}                     * +/- each of these values forms boundary around subgoal
                     'onpt': []},                               * Not yet implemented

                    {'subgoal': 2, ...
                    ...}]
    Attributes:
        mode (int): see above
        subgoal_model (obj):
        endpoints (list, floats): endpoints for the current task
        confidence (float): probability that current task is nominal
        blend_threshold (float): mininum confidence to initiate blending
        alpha (float): BSC blending parameter alpha
        subgoal: current triggered subgoal
        active: assistance active
        history: primitives and endpoints from recent history (window TBD)
    '''
    def __init__(self, mode, sg_model, alpha=0):
        self.mode = mode
        if self.mode == 0:  # Blending off
            self.alpha = 0
        elif self.mode == 1:  # Static alpha subgoal predictive
            self.alpha = alpha
        # elif self.mode == 2:
            #  We will see what goes here

        self.subgoal = 0            # Subgoal 0 denotes no subgoal to start
        self.active = False         # Active is bool, False means no assistance to start
        self.sg_model = sg_model

        # Important: build a list of the subgoals for iterating
        self.sg_list = [self.sg_model[i]['subgoal'] for i in range(len(self.sg_model))]

    def update_state(self, js_inputs, ms_values):
        '''Poll event triggers and update subgoal and/or active boolean.

        TODO:
            fix sloppy indexing into sg_model, subgoals index from 1

        Args:
            js_inputs (list: float): a list of js_inputs in the form [BM, SK, BK, SW]
            ms_values (list: float): listed measurement values for each actuator

        Returns:
            subgoal (int)
            active (bool)
        '''
        # Look for a terminating cue
        for sg in self.sg_model:
            # print([abs(ms_values[i] - sg['subgoal_pos'][i]) for i in range(4)])
            # print([(ms_values[i] - sg['subgoal_pos'][i]) < sg['npt'][i] for i in range(4)])

            if [abs(ms_values[i] - sg['subgoal_pos'][i]) < sg['npt'][i] for i in range(4)] == [True]*4:
                self.subgoal = (sg['subgoal'] % len(self.sg_list)) + 1  # i = (i % length) + 1 (some magic)
                self.active = False

        less_than = ((js_inputs[self.sg_model[self.subgoal-1]['it'][0]] < self.sg_model[self.subgoal-1]['it'][1]))
        # print less_than
        negative = ((self.sg_model[self.subgoal-1]['it'][1]) < 0)
        # print negative
        if not (less_than != negative):  # If input < threshold and threshold negative, or > = threshold and threshold positive
            self.active = True

        return self.subgoal, self.active

# DEPRECATED
# class Subgoal():
#     '''Subgoal class contains information about triggers of subgoals.

#     Proximity triggers terminate the subgoal, input triggers initiate the subgoal.

#     Args:
#         subgoal (list: float): the subgoals position in the exc base frame (x, y, z, theta)
#         it (list: floats): input triggers, joystick states that initiate a subgoal
#         npt (list: floats): nominal proximity triggers, mark a nominal task termination
#         onpt (list: floats): off-nominal proximity triggers, mark an off nominal transition
#
#     Attributes:
#         subgoal (int): the integer key corresponding to the subgoal
#         subgoal_pos (list: floats): see above
#         it (list: floats)
#         npt (list: floats)
#         onpt (list: floats)
#         active (bool): whether or not the subgoal is currently triggered (active from initiate to terminate)
#     '''
#     def __init__(self, subgoal, subgoal_pos, it, npt, onpt):
#         self.subgoal = subgoal
#         self.nominal_proximity_triggers = nominal_proximity_triggers
#         self.input_triggers = input_triggers


def parser(received, received_parsed):
    '''Parse joystick data from server_02.py, and convert to float'''
    deadzone = 0.1
    toggle_invert = [1, 1, -1, -1]  # Invert [BM, SK, BK, SW] joystick
    try:
        received = received.translate(None, "[( )]").split(',')
        for axis in range(len(received)):
            if (float(received[axis]) > deadzone) or (float(received[axis]) < -deadzone):
                received_parsed[axis] = float(received[axis])*toggle_invert[axis]
            else:
                received_parsed[axis] = 0
        return received_parsed
    except ValueError:
        print '\nValue Error'
        raise ValueError


def blending_law(operator_input, controller_output, alpha):
    '''Blend inputs according to the following law:

        u_b = u + a*(u' - u)

        where u is the operator input, a is the alpha parameter, and u' is the controller output
    '''
    return operator_input + alpha*(controller_output - operator_input)


def name_date_time(file_name):
    '''Returns string of format __file__ + '_mmdd_hhmm.csv' '''
    n = datetime.datetime.now()
    data_stamp = os.path.basename(file_name)[:-3] + '_' + n.strftime('%m%d_%H%M')+'.csv'
    return data_stamp


def exc_setup():
    '''Start all PWM classes and measurement classes'''
    boom = Servo("P9_22", 4.939, 10.01, 'Boom', 0)
    stick = Servo("P8_13", 4.929, 8.861, 'Stick', 1)
    bucket = Servo("P8_34", 5.198, 10.03, 'Bucket', 2)
    swing = Servo("P9_42", 4.939, 10, 'Swing', 3)
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
