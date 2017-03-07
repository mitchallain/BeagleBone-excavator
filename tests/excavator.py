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
#   * October 24, 2016 - Added homing routine
#   * February 17, 2017 - Cleanup and packaging
#   * March 07, 2017 - Added Predictor and GaussianPredictor class
#   *
#
##########################################################################################

import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
from bbio.libraries.RotaryEncoder import RotaryEncoder
import numpy as np
import time
import pickle
from scipy.stats import mvn
# import datetime
# import os


class Servo():
    '''A generic PWM class

    Args:
        servo_pin (str): String representing BeagleBone pin, ex: 'P9_32'
        duty_min (float): Minimum duty cycle
        duty_max (float): Maximum duty cycle
        actuator_name (str, optional): Used identify actuators
        js_index (int, optional): Index of joystick list corresponding to this actuator
        offset (float): Deadband offset

    Attributes:
        duty_min (float): Minimum duty cycle
        duty_max (float): Maximum duty cycle
        duty_span (float): Difference between duty_max and duty_min
        duty_mid (float): Average of duty_max and duty_min
        actuator_name (str, optional): Used identify actuators
        js_index (int, optional): Index of joystick list corresponding to this actuator
        '''
    def __init__(self, servo_pin, duty_min, duty_max, actuator_name='', js_index=0, offset=0):
        self.duty_min = duty_min
        self.duty_max = duty_max
        self.duty_span = self.duty_max - self.duty_min
        self.duty_mid = ((90.0 / 180) * self.duty_span + self.duty_min)
        self.duty_set = self.duty_mid
        self.actuator_name = actuator_name
        self.js_index = js_index
        self.offset = offset

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
            self.file.write('Time,Boom Cmd,Stick Cmd,Bucket Cmd,Swing Cmd,'
                            'Boom Ms,Stick Ms,Bucket Ms,Swing Ms\n')

        elif self.mode == 2:   # Autonomous mode
            self.file.write('Time,Boom Ms,Stick Ms,Bucket Ms,Swing Ms,'
                            'Boom Cmd,Stick Cmd,Bucket Cmd,Swing Cmd,'
                            'Boom Error,Stick Error,Bucket Error,Swing Error\n')

        elif self.mode == 3:   # Blended mode (Commands, Controllers, Blended, Measurements, Class, Probability)
            self.file.write('Time,Boom Cmd,Stick Cmd,Bucket Cmd,Swing Cmd,'
                            'Boom Ctrl,Stick Ctrl,Bucket Ctrl,Swing Ctrl,'
                            'Boom Blended,Stick Blended,Bucket Blended,Swing Blended,'
                            'Boom Ms,Stick Ms,Bucket Ms,Swing Ms,Class,Confidence,\n')

        elif self.mode == 4:   # Blended mvn mode (Commands, Controllers, Blended, Measurements, Likelihoods, Subgoal, Alpha)
            num_of_sgs = 6
            sg_header = ','.join(['SG%i' % i for i in range(num_of_sgs)])
            self.file.write('Time,Boom Cmd,Stick Cmd,Bucket Cmd,Swing Cmd,'
                            'Boom Ctrl,Stick Ctrl,Bucket Ctrl,Swing Ctrl,'
                            'Boom Blended,Stick Blended,Bucket Blended,Swing Blended,'
                            'Boom Ms,Stick Ms,Bucket Ms,Swing Ms,' +
                            sg_header +
                            ',Class,Confidence,\n')

    def log(self, data_listed):
        self.file.write(','.join(map(str, data_listed))+'\n')


class GaussianPredictor():
    '''Abstract class for predictors, subclass and implement an update method

    Args:
        subgoal_dists (np.array): list of k distributions of m variables

    Attributes:
        last_confirmed (int): index of last known subgoal in subgoal_dists
        alpha (float): blending parameter

    Methods:
        get_alpha(): return self.alpha
        update(): maps the values of subgoal_probability to a blending parameter value
    '''
    def __init__(self, filename='gmm_model_exp.pkl'):
        self.last_confirmed = -1
        self.last_suspected = -1
        self.alpha = 0
        self.subgoal_probability = np.zeros(6)
        self.alpha_threshold = 0.7
        # Model params
        with open(filename, 'rb') as openfile:
            tmp = pickle.load(openfile)
            self.means = tmp['means']
            self.covs = tmp['covs']

    def update(self, state, action):
        self.subgoal_probability = get_mvn_action_likelihood(state, action, self.means, self.covs)[0]
        max_ll = np.max(self.subgoal_probability)
        if (max_ll > self.alpha_threshold):
            self.alpha = lin_map(max_ll, 0.7, 1, 0.5, 0.8)
            self.subgoal = np.argmax(self.subgoal_probability)
        else:
            self.alpha = 0

    def get_target_sg_pos(self):
        return self.means[self.subgoal_ind]

    def check_if_terminated(self, state, threshold=0.8):
        ''' See if state is within termination region, assign last subgoal'''
        termination_probability = np.array([sg.pdf(state) for sg in self.subgoal_dists])
        if (termination_probability > threshold).any():
            self.last_confirmed = np.argmax(termination_probability) + 1


def get_mvn_action_likelihood(states, actions, means, covs):
    ''' Taken from jupyter notebook gaussian-likelihood,
        uses Alan Genz/Enthought Inc.'s multivariate normal Fortran functions in Scipy

    Args:
        states (np.array): m-length state or n x m array of states
        actions (np.array): m-length action or n x m array of actions
        means (np.array): k x m array of means for k subgoals
        covs (np.array): k x m x m array of m covariance matrices for k subgoals

    Returns:
        action_likelihoods (np.array): n x k array of likelihoods for each subgoal for n states

    TODO:
        marginalize inactive variables by dropping covariances instead of computing whole domain
    '''
    if states.shape != actions.shape:
        raise ValueError('state and action args must have equal dimension.')
    elif states.ndim == 1:
        states = np.expand_dims(states, axis=0)
        actions = np.expand_dims(actions, axis=0)
    action_likelihoods = np.zeros((states.shape[0], means.shape[0]))
    indicator = np.zeros(action_likelihoods.shape)
    for i in xrange(states.shape[0]):
        for g in xrange(means.shape[0]):
            low = np.zeros(states.shape[1])
            upp = np.copy(low)
            for j in xrange(states.shape[1]):
                if actions[i, j] < 0:
                    low[j] = means[g, j] - 10 * covs[g, j, j]
                    upp[j] = states[i, j]
                elif actions[i, j] > 0:
                    low[j] = states[i, j]
                    upp[j] = means[g, j] + 10 * covs[g, j, j]
                else:  # Yields probability 1
                    low[j] = means[g, j] - 10 * covs[g, j, j]
                    upp[j] = means[g, j] + 10 * covs[g, j, j]
            # pdb.set_trace()
            action_likelihoods[i, g], indicator[i, g] = mvn.mvnun(low, upp, means[g], covs[g], maxpts=100000)
            if (indicator[i, g] == 1):
                print(low, upp, means[g], covs[g])
    # if (indicator == 1).any():
        # print('mvnun failed: error code 1')
        # print(low, upp, means, covs)
        # raise ArithmeticError('Fortran function mvnun: error code 1')
    return action_likelihoods


class TriggerPrediction():
    '''The trigger prediction class uses task specific event triggers to determine the current subgoal

    Args:
        sg_model (list: dicts): list of subgoal model dicts, see example below
        mode (int): 0 only terminates, 1 is IFAC style
        alpha (float): BSC blending parameter preset when active, can turn off with 0

    Example arg:
        sg_model = [{'subgoal': 1,
                     'it': [3, -0.5]                            * Joystick index 3 (swing) move past halfway left
                     'subgoal_pos': [6.75, 0.91, 9.95, 1.41]    * Over the pile (actuator space coordinates)
                     'npt': [3, 3, 3, 0.2]}                     * +/- each of these values forms boundary around subgoal
                     'onpt': []},                               * Not yet implemented

                    {'subgoal': 2, ...
                    ...}]
    Attributes:
        alpha (float): BSC blending parameter alpha
        subgoal (int): current triggered subgoal
        prev (int): previous subgoal index
        regen (bool): flag to regenerate trajectories
        active (bool): assistance active
    '''
    def __init__(self, sg_model, mode=1, alpha=0):
        self.mode = mode
        self.alpha = alpha
        self.sg_model = sg_model

        self.dispatch = {0: self.update_0,
                         1: self.update_1}

        # self.subgoal = 0            # Subgoal 0 denotes no subgoal to start
        # self.prev = 6
        self.active = False         # Active is bool, False means no assistance to start
        self.regen = True

        # Important: build a list of the subgoals for iterating
        self.sg_list = [self.sg_model[i]['subgoal'] for i in range(len(self.sg_model))]

    def update(self, state, action):
        ''' General update dispatch function
        Args:
            state (np.array): m-length array of measurements
            action (np.array): m-length array of normalized (and deadbanded) inputs
        '''
        self.dispatch[self.mode](state, action)

    def update_0(self, state, action):
        ''' Mode 0: only terminates'''
        # Look for a terminating cue
        for sg in self.sg_model:
            # Are we in a termination set?
            termination = ([abs(state[i] - sg['subgoal_pos'][i]) < sg['npt'][i] for i in range(4)] == [True]*4)

            # Is this termination set different from our previous subgoal termination?
            # I.e., we don't want to reterminate in the same set over and over.
            different = (sg['subgoal'] != self.prev)

            if termination and different:
                print('Terminated: ', sg['subgoal'])
                self.prev = sg['subgoal']
                self.subgoal = (sg['subgoal'] % len(self.sg_list)) + 1  # i = (i % length) + 1 (some magic)
                self.regen = True
                self.active = False

    def update_1(self, state, action):
        ''' Mode 1: IFAC style FSM predictor '''

        # Look for a terminating cue
        for sg in self.sg_model:
            # Are we in a termination set?
            termination = ([abs(state[i] - sg['subgoal_pos'][i]) < sg['npt'][i] for i in range(4)] == [True]*4)

            # Is this termination set different from our previous subgoal termination?
            # I.e., we don't want to reterminate in the same set over and over.
            different = (sg['subgoal'] != self.prev)

            if termination and different:
                print('Terminated: ', sg['subgoal'])
                self.prev = sg['subgoal']
                self.subgoal = (sg['subgoal'] + 1) % len(self.sg_list)  # i = (i % length) + 1 (some magic)
                self.regen = True
                self.active = False

        less_than = ((action[self.sg_model[self.subgoal-1]['it'][0]] < self.sg_model[self.subgoal-1]['it'][1]))
        # print less_than
        negative = ((self.sg_model[self.subgoal-1]['it'][1]) < 0)
        # print negative
        if not (less_than != negative):  # If input < threshold and threshold negative, or > = threshold and threshold positive
            self.active = True
            print(self.subgoal, 'Ass: True')

        return self.subgoal, self.active


def parse_joystick(received, received_parsed):
    '''Parse joystick data from server_02.py, and convert to float'''
    deadzone = 0.2
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


def blending_law(operator_input, controller_output, alpha, offset=0):
    '''Blend inputs according to the following law:

        u_b = u + a*(u' - u)

        where u is the operator input, a is the alpha parameter, and u' is the controller output

        add offset to eliminate deadband
    '''
    ub = operator_input + alpha*(controller_output - operator_input)
    return (1 - offset) * ub + np.sign(ub) * offset


def lin_map(x, a, b, u, v):
    ''' Maps x from interval (A, B) to interval (a, b)
    TODO: make multidimensional'''
    return (x - a) * ((v - u) / (b - a)) + a


# def name_date_time(file_name):
#     '''Returns string of format __file__ + '_mmdd_hhmm.csv' '''
#     n = datetime.datetime.now()
#     data_stamp = os.path.basename(file_name)[:-3] + '_' + n.strftime('%m%d_%H%M')+'.csv'
#     return data_stamp


def exc_setup():
    '''Start all PWM classes and measurement classes'''
    boom = Servo("P9_22", 4.939, 10.01, 'Boom', 0, 0.5)
    stick = Servo("P8_13", 4.929, 9, 'Stick', 1, 0.5)
    bucket = Servo("P8_34", 5.198, 10.03, 'Bucket', 2, 0.5)
    swing = Servo("P9_42", 4.939, 10, 'Swing', 3, 0)
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


def homing(actuators, measurements, controllers, home, error=[0, 0, 0, 0], dur=0):
    '''Homes all actuators to home position using measurement list and controller list.

    Args:
        actuators (list: Servo): list of Servo objects to send PWM signals to
        measurements (list: Measurement): list of Measurement objects to read position from
        controllers (list: PID): list of PID objects for computing control action
        home (list: floats): list of positions to seek

    Returns:
        None

    TODO: extend function to arbitrary num of actuators by adding endpoint error condition arg

    NOTE: All input lists must be of the same length
    '''
    dim = len(actuators)
    if [dim]*3 != [len(measurements), len(controllers), len(home)]:
        raise ValueError

    for i in range(dim):
        controllers[i].setPoint(home[i])
        controllers[i].update(measurements[i].value)

    start = time.time()
    run_time = 0

    try:
        while [(abs(home[i] - measurements[i].value) > error[i]) for i in range(dim)] == [True]*dim or (run_time < dur):
        # while np.linalg.norm([controllers[i].getError() for i in range(2)]+[controllers[3].getError()*30]) > 1:    # 1 cm radius ball about endpoint
            # Measurement
            for m in measurements:
                m.update_measurement()
                # print(m.value)

            for a, m, c in zip(actuators, measurements, controllers):
                    # Update actuators with control action
                    a.duty_set = c.update(m.value) + a.duty_mid

            # Update PWM, saturation implemented in Servo class
            for a in actuators:
                a.update_servo()
                # print(a.actuator_name + ': ' + str(a.duty_set))
            # print('Error' + str(np.linalg.norm([controllers[i].getError() for i in range(2)]+[controllers[3].getError()*30])))
            run_time = time.time() - start

        for a in actuators:  # Reset all duty cycles after homing
            a.duty_set = a.duty_mid
            a.update_servo()

    except KeyboardInterrupt:
        print '\nClosing PWM signals...'
        for a in actuators:
            a.duty_set = a.duty_mid
            a.update_servo()
        time.sleep(1)
        for a in actuators:
            a.close_servo()
# END HOMING


def saturate(x, lower, upper):
    return max(lower, min(x, upper))
