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
#   * April 10, 2017 - optimized potentiometer interpolation
#   *
#
# Todo:
#   * Rewrite DataLogger() with logging module for speed
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
# from scipy.stats import multivariate_normal
import math
# import datetime
# import os
import pdb


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
    def __init__(self, servo_pin, duty_min, duty_max, actuator_name='', js_index=0,
                 offset=0, bounds=(-20, 20)):
        self.duty_min = duty_min
        self.duty_max = duty_max
        self.duty_span = self.duty_max - self.duty_min
        self.duty_mid = ((90.0 / 180) * self.duty_span + self.duty_min)
        self.duty_set = self.duty_mid
        self.command = 0
        self.actuator_name = actuator_name
        self.js_index = js_index
        self.offset = offset
        self.bounds = bounds
        self.bound_span = bounds[1] - bounds[0]

        self.servo_pin = servo_pin
        print 'starting servo PWM'
        PWM.start(self.servo_pin, self.duty_mid, 50.625)

    def update_servo(self):
        '''Saturate duty cycle at limits'''
        self.duty_set = max(self.duty_min, min(self.duty_max, self.duty_set))
        PWM.set_duty_cycle(self.servo_pin, self.duty_set)

    def update_with_command(self):
        ''' Use normalized command attribute to set duty cycle'''
        self.command = np.clip(self.command, -1, 1)
        self.duty_set = self.duty_mid + (self.command * self.duty_span / 2.0)
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
    def __init__(self, GPIO_pin, measure_type, units='cm'):
        ADC.setup()
        self.GPIO_pin = GPIO_pin
        self.measure_type = measure_type
        self.lookup = {'boom': np.array([[536.0, 564.0, 590.0, 627.0, 667.0, 704.0,
                                          727.0, 762.0, 793.0, 813.0, 834.0, 849.0,
                                          863.0, 881.0, 893.0, 909.0],  # BM Analog Input
                                         [0, 7.89, 14.32, 22.64, 33.27, 47.4, 56.5,
                                          66.26, 75.85, 82.75, 90.5, 95.3, 101.2,
                                          107.7, 112.7, 118.7]]),    # BM Displacement mm
                       'stick': np.array([[554.0, 602.0, 633.0, 660.0, 680.0, 707.0,
                                           736.0, 762.0, 795.0, 820.0, 835.0, 867.0,
                                           892.0, 919.0, 940.0, 959.0, 983.0, 1007.0,
                                           1019.0],                 # SK Analog Input
                                          [0, 11.7, 19.2, 26.2, 31, 38.4, 46.3, 53.7,
                                           63.4, 71.5, 76.3, 87.0, 96.4, 106.1, 114.5,
                                           122.1, 132.2, 143.1, 148.5]]),  # SK Displacement mm
                       'bucket': np.array([[153.0, 187.0, 235.0, 299.0, 346.0, 372.0,
                                            412.0, 440.0, 477.0, 511.0, 529.0, 567.0,
                                            588.0, 602.0, 605.0, 623.0],
                                           [0, 7.28, 15.89, 27.69, 37.3, 43.4, 52.6,
                                            59.63, 68.86, 77.5, 82.5, 92.07, 98.25,
                                            102.3, 103.15, 109.07]])}
        # switch to cm
        if units == 'cm':
            for v in self.lookup.values():
                v[1] /= 10

    def update_measurement(self):
        '''Uses lookup tables and current analog in value to find actuator displacement'''
        self.value = np.interp(ADC.read_raw(self.GPIO_pin), self.lookup[self.measure_type][0], self.lookup[self.measure_type][1])

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
        self.filename = filename
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

        elif self.mode == 3:   # Blended mode (Commands, Controllers, Blended, Measurements, Subgoal, Active)
            self.file.write('Time,Boom Cmd,Stick Cmd,Bucket Cmd,Swing Cmd,'
                            'Boom Ctrl,Stick Ctrl,Bucket Ctrl,Swing Ctrl,'
                            'Boom Blended,Stick Blended,Bucket Blended,Swing Blended,'
                            'Boom Ms,Stick Ms,Bucket Ms,Swing Ms,Subgoal,Active\n')

        elif self.mode == 4:   # Blended mvn mode (Commands, Controllers, Blended, Measurements, Likelihoods, Subgoal, Alpha)
            num_of_sgs = 6
            sg_header = ','.join(['SG%i' % i for i in range(num_of_sgs)])
            self.file.write('Time,Boom Cmd,Stick Cmd,Bucket Cmd,Swing Cmd,'
                            'Boom Ctrl,Stick Ctrl,Bucket Ctrl,Swing Ctrl,'
                            'Boom Blended,Stick Blended,Bucket Blended,Swing Blended,'
                            'Boom Ms,Stick Ms,Bucket Ms,Swing Ms,' +
                            sg_header +
                            ',Subgoal,Alpha\n')

        elif self.mode == 5:  # Velocity tests (manual + flag)
            self.file.write('Total Time,Loop Time,Boom Cmd,Stick Cmd,Bucket Cmd,Swing Cmd,'
                            'Boom Ms,Stick Ms,Bucket Ms,Swing Ms,Flag,Actuator\n')

    def log(self, data_listed):
        self.file.write(','.join(map(str, data_listed))+'\n')

    def close(self):
        ''' Close is coupled to metadata creation to *encourage* trial notes '''
        notes = raw_input('Notes about this trial: ')
        n = open('data/metadata.csv', 'a')
        n.write(self.filename + ',' + notes)
        n.close()
        self.file.close()


class GaussianPredictor():
    '''Abstract class for predictors, subclass and implement an update method

    NOTE: Subgoal numbering now indexes from zero

    Args:
        subgoal_dists (np.array): list of k distributions of m variables

    Attributes:
        last_confirmed (int): index of last subgoal from check_if_terminated()
        last_suspected (int): last subgoal to activate blending
        subgoal (int): current subgoal
        likelihood (np.array): k-length array of action likelihoods
        subgoal_probability (np.array): k-length array of posterior probabilities
        alpha (float): blending parameter
        alpha_threshold (float): threshold to activate blending
        means (np.array): k x m array of subgoal means
        covs (np.array): k x m x m array of subgoal covariances
        trans (np.array): k x k stochastic transition matrix
        kdim (np.array): number of subgoals inferred from the above vars

    Methods:
        update(): maps the values of subgoal_probability to a blending parameter value
        get_target_sg_pos(): returns location of current subgoal
        check_if_terminated(): checks if current state is within subgoal dist
                               with confidence over threshold
    '''
    def __init__(self, filename='gmm_model_exp.pkl'):
        self.last_confirmed = -1
        self.last_suspected = -1
        self.subgoal = -1
        self.alpha = 0
        self.alpha_threshold = 0.7
        # Model params
        with open(filename, 'rb') as openfile:
            tmp = pickle.load(openfile)
            self.means = tmp['means']
            self.covs = tmp['covs']
            self.trans = tmp['trans']
            # self.trans = np.ones((6, 6))
            self.queus = tmp['queues']
        self.kdim = len(self.means)
        self.subgoal_probability = np.zeros(self.kdim)

    def update(self, state, action):
        self.check_if_terminated(state)
        self.likelihood = get_mvn_action_likelihood_marginal(state, action, self.means, self.covs)[0]

        # Apply transition vector corresponding to last confirmed sg
        self.subgoal_probability = self.trans[:, self.last_confirmed] * self.likelihood

        # Normalize posterior
        self.subgoal_probability = np.nan_to_num(self.subgoal_probability / np.sum(self.subgoal_probability))
        # self.subgoal_probability = np.nan_to_num(self.subgoal_probability)

        MAP = np.max(self.subgoal_probability)
        if (MAP > self.alpha_threshold):
            self.alpha = lin_map(MAP, self.alpha_threshold, 1, 0.3, 0.6)
            self.subgoal = np.argmax(self.subgoal_probability)
        else:
            self.alpha = 0

    def get_target_sg_pos(self):
        return self.means[self.subgoal]

    def check_if_terminated(self, state, threshold=0.6):
        ''' See if state is within termination region, assign last subgoal'''
        termination_probability = np.array([multivariate_normal(self.means[i], self.covs[i]).pdf(state) for i in xrange(self.kdim)])
        if (termination_probability > threshold).any():
            self.last_confirmed = np.argmax(termination_probability)

    def check_if_terminated_update_stats(self, state, action, threshold=0.001):
        ''' Checks if we are in a subgoal, which is defined as being within a
            (NEW) subgoal distribution (VALID), and having zero velocity (STILL)

            If all three conditions True, then add to queue and recompute stats
            Can suppress self.update_stats() for static distributions

            Todo: revise these conditions (is zero velocity appropriate?)
            '''
        termination_probability = np.array([multivariate_normal(self.means[i], self.covs[i]).pdf(state) for i in xrange(self.kdim)])
        NEW = (np.argmax(termination_probability) != self.last_confirmed)
        VALID = (termination_probability > threshold).any()
        STILL = (action == 0).all()
        if VALID and STILL and NEW:
            # Set last_confirmed to sg index
            self.last_confirmed = np.argmax(termination_probability)
            # Add location to corresponding queue
            self.queues[self.last_confirmed].append(state)
            self.update_stats()

    def update_stats(self):
        ''' Recalculate stats for last confirmed'''
        self.means[self.last_confirmed] = np.mean(self.queues[self.last_confirmed], axis=0)
        self.covs[self.last_confirmed] = np.cov(np.array(self.queues[self.last_confirmed]).T)


def get_mvn_action_likelihood_marginal_mvndst(states, actions, means, covs):
    ''' Rewriting the original multivariate action likelihood to marginalize out inactive vars
        uses Alan Genz's multivariate normal Fortran function 'mvndst' in Scipy

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

    # For state, action pair index i
    for i in xrange(states.shape[0]):
        # Find active axes and skip if null input
        active = np.where(actions[i] != 0)[0]
        if active.size == 0:
            break

        # Else, compute mvn pdf integration for each subgoal
        # Bounds are shifted so that dist is zero mean
        for g in xrange(means.shape[0]):
            low = np.copy(states[i] - means[g])
            upp = np.copy(states[i] - means[g])
            infin = np.zeros(actions.shape[1])

            # Iterate through active indices and set low and upper bounds of ATD action-targeted domain
            # infin is an integer code used by func mvndst.f
            for j in xrange(actions.shape[1]):
                if actions[i, j] < 0:  # Negative action
                    infin[j] = 0
                elif actions[i, j] > 0:  # Postive action
                    infin[j] = 1
                else:
                    infin[j] = -1

            # Marginalize out inactive variables by dropping means and covariances
            corr = pack_covs(covs[g])
            # logging.info('Correlation coeff: %s \n'
            #              'Covariance matrix: %s \n'
            #              'Active: %s' % (corr, covs, active))

            _, action_likelihoods[i, g], indicator[i, g] = mvn.mvndst(low, upp, infin, corr)

            # if (indicator[i, g] == 1):
            #     logging.error('mvn.mvndst() failed with args: \n'
            #                   'low: %s \n upp: %s \n'
            #                   'infin: %s \n corr: %s \n' % (low, upp, infin, corr))
    return action_likelihoods


def pack_covs(covs):
    ''' To support Alan Genz's mvndst function; go read the documentation '''
    d = len(covs)
    corr = np.zeros((d*(d-1)/2))
    for i in range(d):
        for j in range(d):
            if (i > j):
                corr[j + ((i-1) * i) / 2] = covs[i, j]/(math.sqrt(covs[i, i] * covs[j, j]))
    return corr


# def cov_to_corr(cov):
#     corr = np.zeros(cov.shape)
#     for (i, j), val in [(i, j) for i in range(cov.shape[0]) for j in range(cov.shape[1])]:
#         corr[i, j] = cov[i, j] / math.sqrt(cov[i, i] * cov[j, j])
#     return corr


def get_mvn_action_likelihood_marginal(states, actions, means, covs):
    ''' Rewriting the original multivariate action likelihood to marginalize out inactive vars
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

    # For state, action pair index i
    for i in xrange(states.shape[0]):
        # Find active axes and skip if null input
        active = np.where(actions[i] != 0)[0]
        if active.size == 0:
            break

        # Else, compute mvn pdf integration for each subgoal
        for g in xrange(means.shape[0]):
            low = np.zeros(active.shape)
            upp = np.copy(low)

            # Iterate through active indices and set low and upper bounds of ATD action-targeted domain
            # Bounds at +/-10 sig figs because no option for infinite, check this is sufficient b/c skew
            for num, j in enumerate(active):
                if actions[i, j] < 0:  # Negative action
                    low[num] = means[g, j] - 10 * covs[g, j, j]
                    upp[num] = states[i, j]
                else:  # Postive action
                    low[num] = states[i, j]
                    upp[num] = means[g, j] + 10 * covs[g, j, j]

            # Marginalize out inactive variables by dropping means and covariances
            means_marg = means[g][active]
            covs_marg = covs[g][active][:, active]
            # pdb.set_trace()
            action_likelihoods[i, g], indicator[i, g] = mvn.mvnun(low, upp, means_marg, covs_marg, maxpts=100000)

            if (indicator[i, g] == 1):
                print(low, upp, means_marg, covs_marg)
    # if (indicator == 1).any():
        # print('mvnun failed: error code 1')
        # print(low, upp, means, covs)
        # raise ArithmeticError('Fortran function mvnun: error code 1')
    return action_likelihoods


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

        self.subgoal = 0            # Subgoal 0 denotes no subgoal to start
        self.prev = 6
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
        return self.dispatch[self.mode](state, action)

    def update_0(self, state, action):
        ''' Mode 0: only terminates, no active'''
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
        # pdb.set_trace()
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
        else:  # No input
            self.active = False

        return self.subgoal, self.active


def parse_joystick(received, received_parsed):
    '''Parse joystick data from server_02.py, and convert to float'''
    deadzone = 0.2
    toggle_invert = [1, 1, -1, -1]  # Invert [BM, SK, BK, SW] joystick
    try:
        received = received.translate(None, "[( )]").split(',')
        for axis in xrange(len(received)):
            if (float(received[axis]) > deadzone) or (float(received[axis]) < -deadzone):
                received_parsed[axis] = float(received[axis])*toggle_invert[axis]
            else:
                received_parsed[axis] = 0
        return received_parsed
    except ValueError:
        print '\nValue Error'
        raise ValueError


def parse_joystick_trigger(received, received_parsed):
    '''Rewritten to handle trigger input, see excavator.parse_joystick'''
    deadzone = 0.2
    toggle_invert = [1, 1, -1, -1]  # Invert [BM, SK, BK, SW] joystick
    try:
        received = received.translate(None, "[( )]").split(',')
        for axis in xrange(len(received) - 1):
            if (float(received[axis]) > deadzone) or (float(received[axis]) < -deadzone):
                received_parsed[axis] = float(received[axis])*toggle_invert[axis]
            else:
                received_parsed[axis] = 0
        return received_parsed[0:4], int(received[4])
    except ValueError:
        print '\nValue Error'
        raise ValueError


def blending_law(operator_input, controller_output, alpha, index=0, offset=0, oppose=False, swing=False):
    '''Blend inputs according to the following law:

                    u_b = u + a*(u' - u)

        where u is the operator input, a is the alpha parameter,
        and u' is the controller output

        Args:
            operator_input (float)
            controller_output (float)
            alpha (float): alpha parameter
            index (int): actuator index
            offset (float): eliminate deadband, maps output [0, 1] to [offset, 1]
            oppose (bool): if True, allow assistance to oppose operator
            swing (bool): if False, swing function is unblended

        Returns:
            blended output ub (mapped to offset)
    '''
    if (not oppose) and (np.sign(operator_input) * np.sign(controller_output) == -1):
        ub = operator_input
    elif (not swing) and (index == 3):
        ub = operator_input
    else:
        ub = operator_input + alpha * (controller_output - operator_input)
    return (1 - offset) * ub + np.sign(ub) * offset


def lin_map(x, a, b, u, v):
    ''' Maps x from interval (A, B) to interval (a, b)
    TODO: make multidimensional'''
    return (x - a) * ((v - u) / (b - a)) + u


# def name_date_time(file_name):
#     '''Returns string of format __file__ + '_mmdd_hhmm.csv' '''
#     n = datetime.datetime.now()
#     data_stamp = os.path.basename(file_name)[:-3] + '_' + n.strftime('%m%d_%H%M')+'.csv'
#     return data_stamp

def exc_setup():
    '''Start all PWM classes and measurement classes'''
    boom = Servo("P9_22", duty_min=4.5, duty_max=9.5, actuator_name='Boom',
                 js_index=0, offset=0.5, bounds=(0, 118.7))
    stick = Servo("P8_13", duty_min=4.5, duty_max=9.5, actuator_name='Stick',
                  js_index=1, offset=0.5, bounds=(0, 148.5))
    bucket = Servo("P8_34", duty_min=5.0, duty_max=10.0, actuator_name='Bucket',
                   js_index=2, offset=0.5, bounds=(0, 109.07))
    swing = Servo("P9_42", duty_min=5.0, duty_max=10.0, actuator_name='Swing',
                  js_index=3, offset=0, bounds=(-0.35, 1.92))  # (-10, 110) deg
    actuators = [boom, stick, bucket, swing]

    # Initialize Measurement classes for string pots
    ADC.setup()
    boom_ms = Measurement('P9_37', 'boom')
    stick_ms = Measurement('P9_33', 'stick')
    bucket_ms = Measurement('P9_35', 'bucket')
    swing_ms = Encoder()
    measurements = [boom_ms, stick_ms, bucket_ms, swing_ms]
    return actuators, measurements


def actuator_setup():
    '''Start all PWM classes'''
    boom = Servo("P9_22", 4.5, 9.5, 'Boom', 0, 0.5)
    stick = Servo("P8_13", 4.5, 9.5, 'Stick', 1, 0.5)
    bucket = Servo("P8_34", 5.0, 10.0, 'Bucket', 2, 0.5)
    swing = Servo("P9_42", 5.0, 10.0, 'Swing', 3, 0)
    return [boom, stick, bucket, swing]


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


def close_io(socket, actuators):
    print '\nClosing PWM signals...'
    socket.close()
    for a in actuators:
        a.duty_set = a.duty_mid
        a.update_servo()
    time.sleep(1)
    for a in actuators:
        a.close_servo()


def safe_action(measurements, actuators):
    ''' Replaces method update_servo() of Servo class, enforces saturation and safe limits

    Safe limits enforces action direction within 5% of actuator attribute bounds'''
    for m, a in zip(measurements, actuators):
        if (((m.value - a.bounds[0]) / a.bound_span) < 0.05):
            # Enforce positive action
            a.command *= np.clip(a.command, 0, 1)
        elif (((m.value - a.bounds[1]) / a.bound_span) > -0.05):
            # Enforce negative action
            a.command = np.clip(a.command, -1, 0)
        a.update_with_command()


def check_in_bounds(measurements, actuators, margin=0.05):
    ''' Returns boolean corresponding to whether machine pose is in bounds '''
    for m, a in zip(measurements, actuators):
        if (((m.value - a.bounds[0]) / a.bound_span) < margin) or (((m.value - a.bounds[1]) / a.bound_span) > -margin):
            return False
    return True
