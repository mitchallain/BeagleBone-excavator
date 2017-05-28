#!/bin/env python

import math
import numpy as np
import mat4py
import pdb
import time
import datetime
# from data_logging import DataLogger
from sg_model_for_sim import sg_model
# from subgoal_dists import sg_dists
# from scipy.stats import mvn
# import pickle
import excavator as exc
import exc_analysis.prediction as pred
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
    def __init__(self, js1, js2, order, params):
        Wireframe.__init__(self)

        # Attach joysticks
        self.js1 = js1
        self.js2 = js2

        # Store attributes
        self.exc = mat4py.loadmat('exc.mat')['exc']
        self.params = params
        self.order = order

        self.abs_start = self.loop_start = time.time()

        # Initialize state and drawing nodes
        self.state = np.array([7.0, 5, 4, 0])  # Initialize to position over bucket
        self.addNodes([(0, 0, 0)]*6)  # Six pointless nodes will be moved next step
        self.forward_kin()
        self.addEdges([(n, n+1) for n in range(0, 5)])

        # Initialize logger if log=True
        if params['logging']:
            self.dl = exc.DataLogger(4, exc.name_date_time('sim'))

        # Check blending scheme
        if params['blending'] == 'off':
            self.alpha = 0
        elif params['blending'] == 'static':
            self.alpha = 0.5

        # Init predictor, NOTE update methods must have identical APIs
        if params['prediction'] == 'fsmp':
            self.p = pred.TriggerPrediction(sg_model)
        elif params['prediction'] == 'acp':
            self.p = pred.ActionCompPredictor('gmm_model_sim.pkl')
        elif params['prediction'] == 'mvnacp':
            self.p = pred.GaussianPredictor('gmm_model_sim.pkl')

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
        self.p.update(self.state, self.js)

        if self.params['blending'] != 'off':
            self.update_controllers()
            self.blending()
        else:
            # direct control
            self.vels = self.js * np.array([2, 2, 2, 1])

        # Forward Euler with actuator states, then kinematics for new pose
        self.forward_euler(time.time() - self.loop_start)
        self.loop_start = time.time()
        self.forward_kin()

        if self.params['logging']:
            # Log with DataLogger()
            self.dl.log([time.time() - self.abs_start] + self.js.tolist() +
                        self.pid.tolist() + self.command.tolist() +
                        self.state.tolist() + self.p.subgoal_probability.tolist() +
                        [self.p.subgoal, self.p.alpha])

        # Fix orientation
        self.orient()

        while (time.time() - self.loop_start) < 0.02:  # 50 Hz
            pass

    def get_joysticks(self):
        js_input = np.array([self.js1.get_axis(0), self.js1.get_axis(1), self.js2.get_axis(0), self.js2.get_axis(1)])

        # Apply deadzone
        js_input[(js_input >= -0.3) & (js_input <= 0.3)] = 0

        # Permute and flip direction
        self.js = js_input[self.order]*np.array([1, 1, -1, -1])

    def update_controllers(self):
        # If active and new
        if (self.p.alpha > 0) and (self.p.last_suspected != self.p.subgoal):
            for i, c in enumerate(self.controllers):
                c.setIntegrator(0)
                c.setDerivator(0)
                c.setPoint(self.p.get_target_sg_pos()[i])

        self.p.last_suspected = self.p.subgoal

    def blending(self):
        self.pid = np.zeros((4))
        for i, c in enumerate(self.controllers):
            self.pid[i] = c.update_sat(self.state[i])
            self.command[i] = exc.blending_law(self.js[i], self.pid[i], self.p.alpha, index=i)
        # Scale to velocity
        self.vels = self.command * np.array([2, 2, 2, 1])
        self.perturb = self.p.alpha * (self.pid - self.js)  # Might be an error here

    def orient(self):
        self.rotateX((0, 0, 0), 1.57)
        self.rotateY((0, 0, 0), 0.7853)
        self.rotateX((0, 0, 0), 2.8)
        self.translate('x', 400)
        self.translate('y', 300)


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
