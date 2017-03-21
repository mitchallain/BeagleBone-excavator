#!/bin/env python

import math
import numpy as np
import mat4py
import pdb
import time
import datetime
# from data_logging import DataLogger
# from sg_model import sg_model
# from subgoal_dists import sg_dists
# from scipy.stats import mvn
# import pickle
import excavator as exc
from PID import PID


class Node:
    def __init__(self, coordinates):
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.z = coordinates[2]

    def __repr__(self):
        return '%s(%.2f, %.2f, %.2f)' % (self.__class__, self.x, self.y, self.z)

    def move(self, coordinates):
        self.x = coordinates[0]
        self.y = coordinates[1]
        self.z = coordinates[2]


class Edge:
    def __init__(self, start, stop):
        self.start = start
        self.stop = stop


class Wireframe:
    def __init__(self):
        self.nodes = []
        self.edges = []
        self.dynamic = False

    def addNodes(self, nodeList):
        for node in nodeList:
            self.nodes.append(Node(node))

    def addEdges(self, edgeList):
        for (start, stop) in edgeList:
            self.edges.append(Edge(self.nodes[start], self.nodes[stop]))

    def outputNodes(self):
        print "\n --- Nodes --- "
        for i, node in enumerate(self.nodes):
            print " %d: (%.2f, %.2f, %.2f)" % (i, node.x, node.y, node.z)

    def outputEdges(self):
        print "\n --- Edges --- "
        for i, edge in enumerate(self.edges):
            print " %d: (%.2f, %.2f, %.2f)" % (i, edge.start.x, edge.start.y, edge.start.z),
            print "to (%.2f, %.2f, %.2f)" % (edge.stop.x,  edge.stop.y,  edge.stop.z)

    def translate(self, axis, d):
        """ Add constant 'd' to the coordinate 'axis' of each node of a wireframe """

        if axis in ['x', 'y', 'z']:
            for node in self.nodes:
                setattr(node, axis, getattr(node, axis) + d)

    def scale(self, (centre_x, centre_y), scale):
        """ Scale the wireframe from the centre of the screen """

        for node in self.nodes:
            node.x = centre_x + scale * (node.x - centre_x)
            node.y = centre_y + scale * (node.y - centre_y)
            node.z *= scale

    def findCentre(self):
        """ Find the centre of the wireframe. """

        num_nodes = len(self.nodes)
        meanX = sum([node.x for node in self.nodes]) / num_nodes
        meanY = sum([node.y for node in self.nodes]) / num_nodes
        meanZ = sum([node.z for node in self.nodes]) / num_nodes

        return (meanX, meanY, meanZ)

    def rotateX(self, (cx,cy,cz), radians):
        for node in self.nodes:
            y      = node.y - cy
            z      = node.z - cz
            d      = math.hypot(y, z)
            theta  = math.atan2(y, z) + radians
            node.z = cz + d * math.cos(theta)
            node.y = cy + d * math.sin(theta)

    def rotateY(self, (cx,cy,cz), radians):
        for node in self.nodes:
            x      = node.x - cx
            z      = node.z - cz
            d      = math.hypot(x, z)
            theta  = math.atan2(x, z) + radians
            node.z = cz + d * math.cos(theta)
            node.x = cx + d * math.sin(theta)

    def rotateZ(self, (cx,cy,cz), radians):        
        for node in self.nodes:
            x      = node.x - cx
            y      = node.y - cy
            d      = math.hypot(y, x)
            theta  = math.atan2(y, x) + radians
            node.x = cx + d * math.cos(theta)
            node.y = cy + d * math.sin(theta)


class ExcWireframe(Wireframe):
    def __init__(self, js1, js2, order, log=False):
        Wireframe.__init__(self)

        # Attach joysticks
        self.js1 = js1
        self.js2 = js2

        # Store attributes
        self.exc = mat4py.loadmat('exc.mat')['exc']
        self.log = log
        self.order = order

        self.abs_start = self.loop_start = time.time()

        # Initialize state and drawing nodes
        self.state = np.array([7.0, 5, 4, 0])  # Initialize to position over bucket
        self.addNodes([(0, 0, 0)]*6)  # Six pointless nodes will be moved next step
        self.forward_kin()
        self.addEdges([(n, n+1) for n in range(0, 5)])

        # Initialize logger if log=True
        if log:
            self.dl = exc.DataLogger(4, exc.name_date_time('sim'))

        self.gp = exc.GaussianPredictor()

        # PID Controllers
        boom_PI = PID(1, 0.1, 0, 0, 0, 4, -4)
        stick_PI = PID(1.5, 0.1, 0, 0, 0, 4, -4)
        bucket_PI = PID(1, 0.1, 0, 0, 0, 4, -4)
        swing_PI = PID(10, 0.1, 0, 0, 0, 4, -4)
        self.controllers = [boom_PI, stick_PI, bucket_PI, swing_PI]
        self.command = np.zeros((4))

    def forward_euler(self, dt):
        self.state = np.clip(self.state + self.vels*dt, [0, 0, 0, 0], [11.8, 12.8, 10.9, 1.7])  # update these

    def forward_kin(self):
        t1 = self.state[3]
        # Define lengths
        a1 = self.exc['a1']
        a2 = self.exc['a2']
        a3 = self.exc['a3']
        a4 = self.exc['a4']
        # Compute or Get joint angles
        # Boom angle
        r_c1 = self.state[0] + self.exc['r_cyl1']
        a_a1b = np.arccos((self.exc['r_o1b']**2 + self.exc['r_o1a']**2 - r_c1**2)/(2 * self.exc['r_o1b']*self.exc['r_o1a']))
        t2 = a_a1b - self.exc['a_b12'] - self.exc['a_a1x1']

        # Stick angle
        r_c2 = self.state[1] + self.exc['r_cyl2']
        a_c2d = np.arccos((self.exc['r_o2c']**2 + self.exc['r_o2d']**2 - r_c2**2)/(2 * self.exc['r_o2c'] * self.exc['r_o2d']))
        t3 = 3 * np.pi - self.exc['a_12c'] - a_c2d - self.exc['a_d23']

        # Bucket angle
        r_c3 = self.state[2] + self.exc['r_cyl3']
        a_efh = np.arccos((self.exc['r_ef']**2 + self.exc['r_fh']**2 - r_c3**2)/(2 * self.exc['r_ef'] * self.exc['r_fh']))
        a_hf3 = np.pi - self.exc['a_dfe'] - a_efh
        r_o3h = math.sqrt(self.exc['r_o3f']**2 + self.exc['r_fh']**2 - 2 * self.exc['r_o3f'] * self.exc['r_fh'] * np.cos(a_hf3))
        a_f3h = np.arccos((r_o3h**2 + self.exc['r_o3f']**2 - self.exc['r_fh']**2)/(2 * r_o3h * self.exc['r_o3f']))
        a_h3g = np.arccos((r_o3h**2 + self.exc['r_o3g']**2 - self.exc['r_gh']**2)/(2 * r_o3h * self.exc['r_o3g']))
        t4 = 3 * np.pi - a_f3h - a_h3g - self.exc['a_g34'] - self.exc['a_23d']

        # Compact trig funcs
        c1 = np.cos(t1)
        c2 = np.cos(t2)
        c234 = np.cos(t2 + t3 + t4)
        c23 = np.cos(t2 + t3)
        s1 = np.sin(t1)
        s2 = np.sin(t2)
        s234 = np.sin(t2 + t3 + t4)
        s23 = np.sin(t2 + t3)

        # Nodes
        gos = 17.1
        gnd = tuple([0]*3)
        base = (0, 0, gos)
        o1 = (a1*c1, a1*s1, gos)
        o2 = ((a2*c2 + a1)*c1,
              (a2*c2 + a1)*s1,
              a2*s2 + gos)
        o3 = ((a2*c2 + a3*c23 + a1)*c1,
              (a2*c2 + a3*c23 + a1)*s1,
              gos + a2*s2 + a3*s23)
        o4 = ((a4*c234 + a3*c23 + a2*c2 + a1)*c1,
              (a4*c234 + a3*c23 + a2*c2 + a1)*s1,
              gos + a2*s2 + a3*s23 + a4*s234)

        new_nodes = [gnd, base, o1, o2, o3, o4]

        for i in range(6):
            # pdb.set_trace()
            self.nodes[i].move(new_nodes[i])

    def update(self):
        self.get_joysticks()

        # Update prediction
        self.gp.update(self.state, self.js)

        self.update_controllers()

        self.blending()

        # Forward Euler with actuator states, then kinematics for new pose
        self.forward_euler(time.time() - self.loop_start)
        self.loop_start = time.time()
        self.forward_kin()

        if self.log:
            # Log with DataLogger()
            self.dl.log([time.time() - self.abs_start] + self.js.tolist() +
                        self.pid.tolist() + self.command.tolist() +
                        self.state.tolist() + self.gp.subgoal_probability.tolist() +
                        [self.gp.subgoal, self.gp.alpha])

        # Fix orientation
        self.__orient()

        while (time.time() - self.loop_start) < 0.02:
            pass

    def get_joysticks(self):
        js_input = np.array([self.js1.get_axis(0), self.js1.get_axis(1), self.js2.get_axis(0), self.js2.get_axis(1)])

        # Apply deadzone
        js_input[(js_input >= -0.3) & (js_input <= 0.3)] = 0

        # Permute and flip direction
        self.js = js_input[self.order]*np.array([1, 1, -1, -1])

    def update_controllers(self):
        # If active and new
        if (self.gp.alpha > 0) and (self.gp.last_suspected != self.gp.subgoal):
            for i, c in enumerate(self.controllers):
                c.setIntegrator(0)
                c.setDerivator(0)
                c.setPoint(self.gp.get_target_sg_pos()[i])

        self.gp.last_suspected = self.gp.subgoal

    def blending(self):
        self.pid = np.zeros((4))
        for i, c in enumerate(self.controllers):
            self.pid[i] = c.update_sat(self.state[i])
            self.command[i] = exc.blending_law(self.js[i], self.pid[i], self.gp.alpha, index=i)
        # Scale to velocity
        self.vels = self.command * np.array([2, 2, 2, 1])
        self.perturb = self.gp.alpha * (self.pid - self.js)  # Might be an error here

    def __orient(self):
        self.rotateX((0, 0, 0), 1.57)
        self.rotateY((0, 0, 0), 0.7853)
        self.rotateX((0, 0, 0), 2.8)
        self.translate('x', 400)
        self.translate('y', 300)


# class Predictor():
#     '''Abstract class for predictors, subclass and implement an update method

#     Args:
#         subgoal_dists (np.array): list of k distributions of m variables

#     Attributes:
#         last_subgoal (int): index of last known subgoal in subgoal_dists
#         alpha (float): blending parameter

#     Methods:
#         get_alpha(): return self.alpha
#         update_alpha(): maps the values of subgoal_probability to a blending parameter value
#     '''
#     def __init__(self, subgoal_dists):
#         self.last_subgoal = 0
#         self.alpha = 0
#         self.subgoal_probability = np.zeros(6)
#         self.subgoal_dists = subgoal_dists

#     def get_alpha(self):
#         return self.alpha

#     def probability_map_to_alpha(self, threshold=0.7):
#         if (self.subgoal_probability > threshold).any():
#             self.alpha = np.argmax(self.subgoal_probability > 0.7)
#         else:
#             self.alpha = 0

#     def state_check_if_terminated(self, state, threshold=0.8):
#         ''' See if state is within termination region, assign last subgoal'''
#         termination_probability = np.array([sg.pdf(state) for sg in self.subgoal_dists])
#         if (termination_probability > threshold).any():
#             self.last_subgoal = np.argmax(termination_probability) + 1


# class GaussianPredictor(Predictor):
#     ''' Subclass of Predictor class '''
#     def update(self, state, action):
#         self.subgoal_probability = get_mvn_action_likelihood(state, action, self.means, self.covs)[0]


# def get_mvn_action_likelihood(states, actions, means, covs):
#     ''' Taken from jupyter notebook gaussian-likelihood,
#         uses Alan Genz/Enthought Inc.'s multivariate normal Fortran functions in Scipy

#     Args:
#         states (np.array): m-length state or n x m array of states
#         actions (np.array): m-length action or n x m array of actions
#         means (np.array): k x m array of means for k subgoals
#         covs (np.array): k x m x m array of m covariance matrices for k subgoals

#     Returns:
#         action_likelihoods (np.array): n x k array of likelihoods for each subgoal for n states

#     TODO:
#         marginalize inactive variables by dropping covariances instead of computing whole domain
#     '''
#     if states.shape != actions.shape:
#         raise ValueError('state and action args must have equal dimension.')
#     elif states.ndim == 1:
#         states = np.expand_dims(states, axis=0)
#         actions = np.expand_dims(actions, axis=0)
#     action_likelihoods = np.zeros((states.shape[0], means.shape[0]))
#     indicator = np.zeros(action_likelihoods.shape)
#     for i in xrange(states.shape[0]):
#         for g in xrange(means.shape[0]):
#             low = np.zeros(states.shape[1])
#             upp = np.copy(low)
#             for j in xrange(states.shape[1]):
#                 if actions[i, j] < 0:
#                     low[j] = means[g, j] - 10 * covs[g, j, j]
#                     upp[j] = states[i, j]
#                 elif actions[i, j] > 0:
#                     low[j] = states[i, j]
#                     upp[j] = means[g, j] + 10 * covs[g, j, j]
#                 else:  # Yields probability 1
#                     low[j] = means[g, j] - 10 * covs[g, j, j]
#                     upp[j] = means[g, j] + 10 * covs[g, j, j]
#             # pdb.set_trace()
#             action_likelihoods[i, g], indicator[i, g] = mvn.mvnun(low, upp, means[g], covs[g], maxpts=100000)
#             if (indicator[i, g] == 1):
#                 print(low, upp, means[g], covs[g])
#     # if (indicator == 1).any():
#         # print('mvnun failed: error code 1')
#         # print(low, upp, means, covs)
#         # raise ArithmeticError('Fortran function mvnun: error code 1')
#     return action_likelihoods


# def get_mvn_action_likelihood_marginal(states, actions, means, covs):
#     ''' Rewriting the original multivariate action likelihood to marginalize out inactive vars
#         uses Alan Genz/Enthought Inc.'s multivariate normal Fortran functions in Scipy

#     Args:
#         states (np.array): m-length state or n x m array of states
#         actions (np.array): m-length action or n x m array of actions
#         means (np.array): k x m array of means for k subgoals
#         covs (np.array): k x m x m array of m covariance matrices for k subgoals

#     Returns:
#         action_likelihoods (np.array): n x k array of likelihoods for each subgoal for n states

#     TODO:
#         marginalize inactive variables by dropping covariances instead of computing whole domain
#     '''

#     if states.shape != actions.shape:
#         raise ValueError('state and action args must have equal dimension.')

#     elif states.ndim == 1:
#         states = np.expand_dims(states, axis=0)
#         actions = np.expand_dims(actions, axis=0)

#     action_likelihoods = np.zeros((states.shape[0], means.shape[0]))
#     indicator = np.zeros(action_likelihoods.shape)

#     # For state, action pair index i
#     for i in xrange(states.shape[0]):
#         # Find active axes and skip if null input
#         active = np.where(actions[i] != 0)[0]
#         if active.size == 0:
#             break

#         # Else, compute mvn pdf integration for each subgoal
#         for g in xrange(means.shape[0]):
#             low = np.zeros(active.shape)
#             upp = np.copy(low)

#             # Iterate through active indices and set low and upper bounds of ATD action-targeted domain
#             # Bounds at +/-10 sig figs because no option for infinite, check this is sufficient b/c skew
#             for num, j in enumerate(active):
#                 if actions[i, j] < 0:  # Negative action
#                     low[num] = means[g, j] - 10 * covs[g, j, j]
#                     upp[num] = states[i, j]
#                 else:  # Postive action
#                     low[num] = states[i, j]
#                     upp[num] = means[g, j] + 10 * covs[g, j, j]

#             # Marginalize out inactive variables by dropping means and covariances
#             means_marg = means[g][active]
#             covs_marg = covs[g][active][:, active]
#             # pdb.set_trace()
#             action_likelihoods[i, g], indicator[i, g] = mvn.mvnun(low, upp, means_marg, covs_marg, maxpts=100000)

#             if (indicator[i, g] == 1):
#                 print(low, upp, means_marg, covs_marg)
#     # if (indicator == 1).any():
#         # print('mvnun failed: error code 1')
#         # print(low, upp, means, covs)
#         # raise ArithmeticError('Fortran function mvnun: error code 1')
#     return action_likelihoods


# def get_least_likelihood_uncorrelated(state, action, subgoal_dists):
#     '''Computes least likelihood that action belongs to initiation set (see notes) with gaussian sgs

#     Args:
#         state (np.array): m-length machine state
#         action (np.array): m-length (joystick) input with deadzone applied
#         subgoal_dists (list): list of k distributions of m variables

#     Ex: appending subgoal_dists for one subgoal
#         subgoal = []
#         means = np.array([0, 0, 0, 0]); std = np.array([1, 1, 1, 1])
#         subgoal.append(scipy.stats.norm(mean=means, scale=std))

#     Returns:
#         sg_action_likelihood (np.array): k-length likelihood of input action for each possible subgoal
#     '''
#     if (action == 0).all():
#         return np.zeros(6)

#     primitive_likelihoods = np.zeros((6, 4))
#     encoder = {-1: 0,
#                0: 2,
#                1: 1}

#     # Map and encode the js values
#     primitives = np.sign(action)
#     encoded_primitives = np.copy(primitives)
#     for k, v in encoder.iteritems():
#         encoded_primitives[primitives == k] = v

#     for i in range(6):
#         primitive_likelihoods[i] = np.abs(subgoal_dists[i].cdf(state) - encoded_primitives)

#     return np.min(primitive_likelihoods, axis=1)


# class DeterministicPredictor(Predictor):
#     ''' Assume that previous subgoal fully determines next subgoal

#     Args:
#         action (list: float): a list of action in the form [BM, SK, BK, SW]
#         state (list: float): listed measurement values for each actuator
#     '''
#     def __init__(self):
#         Predictor.__init__(self)
#         self.init = [(3, 0.5), (0, -0.5), (1, 0.5), (0, 0.5), (3, -0.5), (2, -0.5)]

#     def update(self, state, action):
#         # Look for a terminating cue
#         for i, sg in enumerate(self.subgoal_dists):
#             # Are we in a termination set?
#             # termination = ([abs(state[i] - sg['subgoal_pos'][i]) < sg['npt'][i] for i in range(4)] == [True]*4)
#             termination = (sg.pdf(state) > 0.16).all()

#             # Is this termination set different from our previous subgoal termination?
#             # I.e., we don't want to reterminate in the same set over and over.
#             different = (i != self.last_subgoal - 1)

#             if termination and different:
#                 self.prev = i + 1
#                 self.subgoal = ((i + 1) % len(self.subgoal_dists)) + 1
#                 self.alpha = 0

#         less_than = (action[self.init[self.subgoal - 1][0]] < self.init[self.subgoal - 1][1])
#         negative = ((self.init[self.subgoal - 1][1]) < 0)
#         if less_than == negative:  # If input < threshold and threshold negative, or > = threshold and threshold positive
#             self.alpha = 0.5


# class TriggerPrediction():
#     '''The trigger prediction class uses task specific event triggers to determine the current subgoal

#     Args:
#         mode (int): 0 is off, 1 is static alpha, 2 is dynamic alpha
#         sg_model (list: dicts): list of subgoal model dicts, see example below
#         alpha (float): BSC blending parameter preset for static mode

#     Example arg sg_model:
#         sg_model = [{'subgoal': 1,
#                      'it': [3, -0.5]                            * Joystick index 3 (swing) move past halfway left
#                      'subgoal_pos': [6.75, 0.91, 9.95, 1.41]    * Over the pile (actuator space coordinates)
#                      'npt': [3, 3, 3, 0.2]}                     * +/- each of these values forms boundary around subgoal
#                      'onpt': []},                               * Not yet implemented

#                     {'subgoal': 2, ...
#                     ...}]
#     Attributes:
#         mode (int): see above
#         subgoal_model (obj):
#         endpoints (list, floats): endpoints for the current task
#         confidence (float): probability that current task is nominal
#         blend_threshold (float): mininum confidence to initiate blending
#         alpha (float): BSC blending parameter alpha
#         subgoal (int): current triggered subgoal
#         prev
#         regen (bool): flag to regenerate trajectories
#         active (bool): assistance active
#         history: primitives and endpoints from recent history (window TBD, not yet implemented)

#     ToDo:
#         numpy and optimize, clean up attributes
#     '''
#     def __init__(self, sg_model, mode=1, alpha=0):
#         self.mode = mode
#         self.alpha = alpha
#         self.sg_model = sg_model
#         # if self.mode == 0:  # Blending off
#         #     self.alpha = 0
#         # elif self.mode == 1:  # Static alpha subgoal predictive
#         #     self.alpha = alpha
#         # elif self.mode == 2:  # Static blending with input responsive on and off states
#         #     self.alpha = alpha
#         # # elif self.mode == 3:
#         self.dispatch = {0: self.update_0,  # Deterministic spatial prediction
#                          1: self.update_1}  # Deterministic with input triggers
#             #  We will see what goes here

#         self.subgoal = 0            # Subgoal 0 denotes no subgoal to start
#         self.prev = 6
#         self.active = False         # Active is bool, False means no assistance to start
#         self.regen = True

#         # Important: build a list of the subgoals for iterating
#         self.sg_list = [self.sg_model[i]['subgoal'] for i in range(len(self.sg_model[0]))]

#     def update(self, js_inputs, ms_values):
#         return self.dispatch[self.mode](js_inputs, ms_values)

#     def update_0(self, js_inputs, ms_values):
#         # Look for a terminating cue
#         for sg in self.sg_model:
#             # Are we in a termination set?
#             termination = ([abs(ms_values[i] - sg['subgoal_pos'][i]) < sg['npt'][i] for i in range(4)] == [True]*4)

#             # Is this termination set different from our previous subgoal termination?
#             # I.e., we don't want to reterminate in the same set over and over.
#             different = (sg['subgoal'] != self.prev)

#             if termination and different:
#                 print('Terminated: ', sg['subgoal'])
#                 self.prev = sg['subgoal']
#                 self.subgoal = (sg['subgoal'] % len(self.sg_list)) + 1  # i = (i % length) + 1 (some magic)
#                 self.regen = True
#                 self.active = False

#     def update_1(self, js_inputs, ms_values):

#         '''Poll event triggers and update subgoal and/or active boolean.

#         TODO:
#             fix sloppy indexing into sg_model, subgoals index from 1

#         Args:
#             js_inputs (list: float): a list of js_inputs in the form [BM, SK, BK, SW]
#             ms_values (list: float): listed measurement values for each actuator

#         Returns:
#             subgoal (int)
#             active (bool)
#         '''

#         # Look for a terminating cue
#         for sg in self.sg_model:
#             # Are we in a termination set?
#             termination = ([abs(ms_values[i] - sg['subgoal_pos'][i]) < sg['npt'][i] for i in range(4)] == [True]*4)

#             # Is this termination set different from our previous subgoal termination?
#             # I.e., we don't want to reterminate in the same set over and over.
#             different = (sg['subgoal'] != self.prev)

#             if termination and different:
#                 print('Terminated: ', sg['subgoal'])
#                 self.prev = sg['subgoal']
#                 self.subgoal = (sg['subgoal'] + 1) % len(self.sg_list)  # i = (i % length) + 1 (some magic)
#                 self.regen = True
#                 self.active = False

#         less_than = ((js_inputs[self.sg_model[self.subgoal-1]['it'][0]] < self.sg_model[self.subgoal-1]['it'][1]))
#         # print less_than
#         negative = ((self.sg_model[self.subgoal-1]['it'][1]) < 0)
#         # print negative
#         if not (less_than != negative):  # If input < threshold and threshold negative, or > = threshold and threshold positive
#             self.active = True
#             print(self.subgoal, 'Ass: True')
#         elif self.mode == 2:
#             self.active = False

#         return self.subgoal, self.active


def create_filename(prefix):
    '''Returns string of format prefix + '_mmdd_hhmm.csv' '''
    n = datetime.datetime.now()
    data_stamp = prefix + '_' + n.strftime('%m%d_%H%M')+'.csv'
    return data_stamp


def forward_kin(state):
    param = mat4py.loadmat('exc.mat')['exc']
    t1 = state[3]
    # Define lengths
    a1 = param['a1']
    a2 = param['a2']
    a3 = param['a3']
    a4 = param['a4']
    # Compute or Get joint angles
    # Boom angle
    r_c1 = state[0] + param['r_cyl1']
    a_a1b = np.arccos((param['r_o1b']**2 + param['r_o1a']**2 - r_c1**2)/(2 * param['r_o1b']*param['r_o1a']))
    t2 = a_a1b - param['a_b12'] - param['a_a1x1']

    # Stick angle
    r_c2 = state[1] + param['r_cyl2']
    a_c2d = np.arccos((param['r_o2c']**2 + param['r_o2d']**2 - r_c2**2)/(2 * param['r_o2c'] * param['r_o2d']))
    t3 = 3 * np.pi - param['a_12c'] - a_c2d - param['a_d23']

    # Bucket angle
    r_c3 = state[2] + param['r_cyl3']
    a_efh = np.arccos((param['r_ef']**2 + param['r_fh']**2 - r_c3**2)/(2 * param['r_ef'] * param['r_fh']))
    a_hf3 = np.pi - param['a_dfe'] - a_efh
    r_o3h = math.sqrt(param['r_o3f']**2 + param['r_fh']**2 - 2 * param['r_o3f'] * param['r_fh'] * np.cos(a_hf3))
    a_f3h = np.arccos((r_o3h**2 + param['r_o3f']**2 - param['r_fh']**2)/(2 * r_o3h * param['r_o3f']))
    a_h3g = np.arccos((r_o3h**2 + param['r_o3g']**2 - param['r_gh']**2)/(2 * r_o3h * param['r_o3g']))
    t4 = 3 * np.pi - a_f3h - a_h3g - param['a_g34'] - param['a_23d']

    # Compact trig funcs
    c1 = np.cos(t1)
    c2 = np.cos(t2)
    c234 = np.cos(t2 + t3 + t4)
    c23 = np.cos(t2 + t3)
    s1 = np.sin(t1)
    s2 = np.sin(t2)
    s234 = np.sin(t2 + t3 + t4)
    s23 = np.sin(t2 + t3)

    # Node
    gos = 17.1
    o4 = ((a4*c234 + a3*c23 + a2*c2 + a1)*c1,
          (a4*c234 + a3*c23 + a2*c2 + a1)*s1,
          gos + a2*s2 + a3*s23 + a4*s234)

    return o4
