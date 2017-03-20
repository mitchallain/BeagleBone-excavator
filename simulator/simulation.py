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
#   * March 10, 2017 -  refactored simtools.py to better utilize excavator.py
#   *                   also,
#
##########################################################################################

import simtools
import pygame
import pdb
import numpy as np
import pickle
import time
import sys
import datetime


# key_to_function = {
#     pygame.K_LEFT:   (lambda x: x.translateAll('x', -10)),
#     pygame.K_RIGHT:  (lambda x: x.translateAll('x',  10)),
#     pygame.K_DOWN:   (lambda x: x.translateAll('y',  10)),
#     pygame.K_UP:     (lambda x: x.translateAll('y', -10)),
#     pygame.K_EQUALS: (lambda x: x.scaleAll(1.25)),
#     pygame.K_MINUS:  (lambda x: x.scaleAll(0.8)),
#     pygame.K_q:      (lambda x: x.rotateAll('X',  0.1)),
#     pygame.K_w:      (lambda x: x.rotateAll('X', -0.1)),
#     pygame.K_a:      (lambda x: x.rotateAll('Y',  0.1)),
#     pygame.K_s:      (lambda x: x.rotateAll('Y', -0.1)),
#     pygame.K_z:      (lambda x: x.rotateAll('Z',  0.1)),
#     pygame.K_x:      (lambda x: x.rotateAll('Z', -0.1))}

def name_date_time_ext(file_name, ext='.pkl'):
    '''Returns string of format file_name + '_mmdd_hhmm.pkl' '''
    try:
        n = datetime.datetime.now()
        file_with_timestamp = file_name + n.strftime('_%m%d_%H%M') + ext
        return file_with_timestamp
    except NameError:
        print('Import datetime')


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
        self.savemode = False
        self.pausemode = True
        self.save = []

    def addWireframe(self, name, wireframe):
        """ Add a named wireframe object. """
        self.wireframes[name] = wireframe

    def run(self):
        """ Create a pygame screen until it is closed. """
        # initialize font; must be called after 'pygame.init()' to avoid 'Font not Initialized' error
        self.myfont = pygame.font.Font(None, 30)

        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    if self.savemode:
                        with open('save.pkl', 'wb') as savefile:
                            pickle.dump(self.save, savefile)
                    running = False
                # Save points for subgoal distribution building
                elif (self.wireframes['exc'].js2.get_button(0) == 1) and self.savemode:
                    self.save.append(self.wireframes['exc'].state)
                    time.sleep(0.5)
                # Pause to examine instant stats
                elif (self.wireframes['exc'].js1.get_button(0) == 1) and self.pausemode:
                    while self.wireframes['exc'].js1.get_button(0) == 1:
                        pygame.event.pump()
                        pass

            # pygame.event.pump()
            self.update()

            self.display()
            # render text
            # print(self.wireframes['exc'].predictor.subgoal)
            self.label = self.myfont.render("Likelihood: %s" % ['%.2f' % elem for elem in self.wireframes['exc'].gp.likelihood.tolist()], 1, (200, 200, 200))
            self.screen.blit(self.label, (20, 20))
            self.label = self.myfont.render("Post: %s" % ['%.2f' % elem for elem in self.wireframes['exc'].gp.subgoal_probability.tolist()], 1, (200, 200, 200))
            self.screen.blit(self.label, (20, 50))
            self.label = self.myfont.render("Subgoal: %i, Ll: %.2f, Alpha: %.2f" % (np.argmax(self.wireframes['exc'].gp.subgoal_probability),
                                                                                    np.max(self.wireframes['exc'].gp.subgoal_probability),
                                                                                    self.wireframes['exc'].gp.alpha), 1, (200, 200, 200))
            self.screen.blit(self.label, (20, 80))
            self.label = self.myfont.render("Operator: %s" % ['%.3f' % elem for elem in self.wireframes['exc'].js.tolist()], 1, (200, 200, 200))
            self.screen.blit(self.label, (20, 110))
            self.label = self.myfont.render("Control: %s" % ['%.3f' % elem for elem in self.wireframes['exc'].pid.tolist()], 1, (200, 200, 200))
            self.screen.blit(self.label, (20, 140))
            self.label = self.myfont.render("Command: %s LC: %i" %
                                            (['%.3f' % elem for elem in self.wireframes['exc'].command],
                                             self.wireframes['exc'].gp.last_confirmed),
                                            1, (200, 200, 200))
            self.screen.blit(self.label, (20, 550))
            pygame.display.flip()

    def update(self):  # read joysticks and apply to excavator
        self.wireframes['exc'].update()
        centre_x = self.width/2
        centre_y = self.height/2
        self.wireframes['exc'].scale((centre_x, centre_y), 6)
        self.wireframes['exc'].translate('y', 200)

    def display(self):
        """ Draw the wireframes on the screen. """

        self.screen.fill(self.background)
        for wf in self.wireframes.values():
            if self.displayEdges:
                for edge in wf.edges:
                    pygame.draw.aaline(self.screen, self.edgeColour, (edge.start.x, edge.start.y), (edge.stop.x, edge.stop.y), 1)

            if self.displayNodes:
                for node in wf.nodes:
                    pygame.draw.circle(self.screen, self.nodeColour, (int(node.x), int(node.y)), self.nodeRadius, 0)

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
    if (len(sys.argv) == 2):
        if (sys.argv[1] == 'log'):
            log = True
    else:
        log = False

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

    # Excavator wireframe, log option from CLI
    exc = simtools.ExcWireframe(js1, js2, order, log=log)
    pv.addWireframe('exc', exc)

    # Truck wireframe
    truck = simtools.Wireframe()
    truck.addNodes([(x, y, z) for x in (37, 67.5) for y in (-9, 9) for z in (0, 15)])
    truck.addEdges([(n, n+1) for n in [0, 2, 4, 6]] + [(n, n+4) for n in range(4)] + [(n, n+2) for n in [0, 1, 4, 5]])
    pv.addWireframe('truck', truck)

    # Pile wireframe
    pile = simtools.Wireframe()
    pile.addNodes([(x, 38, 0) for x in (2, 42)] + [(x, 78, 0) for x in (42, 2)] + [(22, 58, 30)])
    pile.addEdges([(n, 4) for n in range(4)] + [(n, (n + 1) % 4) for n in range(4)])
    pv.addWireframe('pile', pile)

    # Subgoal Nodes
    subgoals = simtools.Wireframe()
    subgoals.addNodes([simtools.forward_kin(mean) for mean in exc.gp.means])
    pv.addWireframe('subgoals', subgoals)

    # Prepare orientation of scene HAS TO MATCH EXC UPDATE FUNC
    pv.rotateAll('X', 1.57)
    pv.rotateAll('Y', 0.7853)
    pv.rotateAll('X', 2.8)
    pv.translateAll('x', 400)
    pv.translateAll('y', 300)
    pv.scaleAll(6)
    pv.translateAll('y', 200)

    pv.run()

    # Shut her down.
    if pv.wireframes['exc'].log:
        pv.wireframes['exc'].dl.close()

    # Save dynamic subgoal locations to pickle
    with open('pickles/' + name_date_time_ext('dyn_sgs'), 'wb') as sg_save:
        pickle.dump(pv.wireframes['exc'].gp.sg_list, sg_save)
