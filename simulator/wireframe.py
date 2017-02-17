#!/bin/env python

import math
import numpy as np
import mat4py
# import numpy.linalg as linalg
import pdb
import time


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
            node.z = centre_y + scale * (node.z - centre_y)
            node.y *= scale

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
    def __init__(self, js1, js2, order):
        Wireframe.__init__(self)
        self.js1 = js1
        self.js2 = js2
        self.order = order
        self.st = time.time()
        self.exc = mat4py.loadmat('exc.mat')['exc']
        self.state = np.array([7.0, 5, 4, 0])  # Initialize to position over bucket
        self.addNodes([(0, 0, 0)]*6)  # Six pointless nodes will be moved next step
        self.forward_kin()
        self.addEdges([(n, n+1) for n in range(0, 5)])

    def forward_euler(self, dt):
        self.state = np.clip(self.state + self.vels*dt, [0, 0, 0, -3.14], [11.8, 12.8, 10.9, 1.57])  # update these

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
        js_input = [self.js1.get_axis(0), self.js1.get_axis(1), self.js2.get_axis(0), self.js2.get_axis(1)]
        self.vels = np.array([js_input[i] for i in self.order])*np.array([2, 2, -2, 1])
        self.vels[(self.vels >= 0.3) & (self.vels <= 0.3)] = 0

        self.forward_euler(time.time() - self.st)
        self.st = time.time()
        self.forward_kin()



# if __name__ == "__main__":
#     cube_nodes = [(x,y,z) for x in (0,1) for y in (0,1) for z in (0,1)]
#     cube = Wireframe()
#     cube.addNodes(cube_nodes)
#     cube.addEdges([(n,n+4) for n in range(0,4)])
#     cube.addEdges([(n,n+1) for n in range(0,8,2)])
#     cube.addEdges([(n,n+2) for n in (0,1,4,5)])

#     cube.outputNodes()
#     cube.outputEdges()
