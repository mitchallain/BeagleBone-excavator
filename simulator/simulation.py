#! /usr/bin/env python

##########################################################################################
# simulation.py
#
# This file uses pygame to simulate the pose of the excavator with actuator velocity inputs
# from joystick and no friction or inertia, using forward kinematics from sim_kin.py
#
# NOTE:
#
# Created: February 16, 2017
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

import wireframe
import pygame
import pdb
import numpy as np

key_to_function = {
    pygame.K_LEFT:   (lambda x: x.translateAll('x', -10)),
    pygame.K_RIGHT:  (lambda x: x.translateAll('x',  10)),
    pygame.K_DOWN:   (lambda x: x.translateAll('y',  10)),
    pygame.K_UP:     (lambda x: x.translateAll('y', -10)),
    pygame.K_EQUALS: (lambda x: x.scaleAll(1.25)),
    pygame.K_MINUS:  (lambda x: x.scaleAll(0.8)),
    pygame.K_q:      (lambda x: x.rotateAll('X',  0.1)),
    pygame.K_w:      (lambda x: x.rotateAll('X', -0.1)),
    pygame.K_a:      (lambda x: x.rotateAll('Y',  0.1)),
    pygame.K_s:      (lambda x: x.rotateAll('Y', -0.1)),
    pygame.K_z:      (lambda x: x.rotateAll('Z',  0.1)),
    pygame.K_x:      (lambda x: x.rotateAll('Z', -0.1))}


class ProjectionViewer:
    """ Displays 3D objects on a Pygame screen """

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption('Wireframe Display')
        self.background = (10, 10, 50)

        self.wireframes = {}
        self.displayNodes = True
        self.displayEdges = True
        self.nodeColour = (255, 255, 255)
        self.edgeColour = (200, 200, 200)
        self.nodeRadius = 4

    def addWireframe(self, name, wireframe):
        """ Add a named wireframe object. """

        self.wireframes[name] = wireframe

    def run(self):
        """ Create a pygame screen until it is closed. """

        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key in key_to_function:
                        key_to_function[event.key](self)

            pygame.event.pump()
            self.update()
            # Prepare orientation of scene
            self.rotateAll('X', 3.1415)
            self.rotateAll('Z', 0.7853)
            self.rotateAll('X', 0.7853)
            self.translateAll('x', 400)
            self.translateAll('z', 300)
            self.scaleAll(6)
            self.translateAll('z', 200)

            # self.rotateAll('Y', 0.8)
            self.display()
            pygame.display.flip()

    def update(self):  # read joysticks and apply to excavator
        self.wireframes['exc'].update()

    def display(self):
        """ Draw the wireframes on the screen. """

        self.screen.fill(self.background)
        for wf in self.wireframes.values():
            if self.displayEdges:
                for edge in wf.edges:
                    pygame.draw.aaline(self.screen, self.edgeColour, (edge.start.x, edge.start.z), (edge.stop.x, edge.stop.z), 1)

            if self.displayNodes:
                for node in wf.nodes:
                    pygame.draw.circle(self.screen, self.nodeColour, (int(node.x), int(node.z)), self.nodeRadius, 0)

    def translateAll(self, axis, d):
        """ Translate all wireframes along a given axis by d units. """

        for wf in self.wireframes.itervalues():
            wf.translate(axis, d)

    def scaleAll(self, scale):
        """ Scale all wireframes by a given scale, centred on the centre of the screen. """

        centre_x = self.width/2
        centre_y = self.height/2

        for wf in self.wireframes.itervalues():
            wf.scale((centre_x, centre_y), scale)

    def rotateAll(self, axis, theta):
        """ Rotate all wireframe about the origin, along a given axis by a given angle. """

        rotateFunction = 'rotate' + axis
        centre = (0, 0, 0)

        for wf in self.wireframes.itervalues():
            # centre = wireframe.findCentre()
            getattr(wf, rotateFunction)(centre, theta)


if __name__ == '__main__':
    pv = ProjectionViewer(800, 600)

    print('Initializing pygame module and joysticks...\n')
    pygame.init()
    pygame.joystick.init()
    js1 = pygame.joystick.Joystick(1)
    js1.init()
    js2 = pygame.joystick.Joystick(0)
    js2.init()

    print('Squeeze trigger on right joystick...')
    while True:
        pygame.event.pump()
        if js1.get_button(0) == 1:
            order = [1, 3, 0, 2]
            break
        elif js2.get_button(0) == 1:
            order = [3, 1, 2, 0]
            break
    print('Right joystick detected.')

    exc = wireframe.ExcWireframe(js1, js2, order)

    pv.addWireframe('exc', exc)

    pv.run()
