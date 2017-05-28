#! /usr/bin/env python

##########################################################################################
# sim.py
#
# Demonstrate different prediction and control modes
#
# NOTE: forked from simlation.py with PEP8 and sys.sargs handling
#       based on ___ by ___
#
# Created: May 26, 2017
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

import simtools2 as simtools
import pygame
import numpy as np
import pickle
import time
import datetime
from sys import argv
# import pdb


params = {'prediction': 'fsmp',
          'blending': 'off',
          'logging': False}

valid = {'prediction': ['fsmp', 'acp', 'mvnacp'],
         'blending': ['off', 'static', 'dynamic']}


def parse_argv(sargs, params):
    ''' Handles sys argument parsing and overwrites defaults in params dict '''
    for i, arg in enumerate(sargs):
        try:
            if arg == '-p':
                if sargs[i + 1] in valid['prediction']:
                    params['prediction'] = sargs[i + 1]
                else:
                    raise NameError('Invalid prediction class.')
            elif arg == '-b':
                if sargs[i + 1] in valid['blending']:
                    params['blending'] = sargs[i + 1]
                else:
                    raise NameError('Invalid blending class.')
            elif arg == '-l':
                params['logging'] = True
        except Exception as e:
            print(str(e) + '\nArgument error. Running with defaults...')

        if params['prediction'] == 'fsmp' and params['blending'] == 'dynamic':
            print('Cannot blend dynamically with FSM prediction. \nUsing static'
                  'blending.')
            params['blending'] = 'static'

    return params


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
        self.display_nodes = True
        self.display_edges = True
        self.node_colour = (255, 255, 255)
        self.edge_colour = (200, 200, 200)
        self.node_radius = 4
        self.savemode = False
        self.pausemode = True
        self.save = []

    def add_wireframe(self, name, wireframe):
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
            ## Prediction Values
            if self.wireframes['exc'].params['prediction'] != 'fsmp':
                self.label = self.myfont.render("Likelihood: %s" % ['%.2f' % elem for elem in self.wireframes['exc'].p.likelihood.tolist()], 1, (200, 200, 200))
                self.screen.blit(self.label, (20, 20))
                self.label = self.myfont.render("Post: %s" % ['%.2f' % elem for elem in self.wireframes['exc'].p.subgoal_probability.tolist()], 1, (200, 200, 200))
                self.screen.blit(self.label, (20, 50))
                self.label = self.myfont.render("Subgoal: %i, P: %.2f, Alpha: %.2f" % (np.argmax(self.wireframes['exc'].p.subgoal_probability),
                                                                                        np.max(self.wireframes['exc'].p.subgoal_probability),
                                                                                        self.wireframes['exc'].p.alpha), 1, (200, 200, 200))

                self.screen.blit(self.label, (20, 80))
                self.label = self.myfont.render("LC: %s, Term Probability: %s" %
                                                (self.wireframes['exc'].p.last_confirmed,
                                                 ['%.5f' % elem for elem in self.wireframes['exc'].p.termination_probability]),
                                                1, (200, 200, 200))
                self.screen.blit(self.label, (20, 550))
            else:
                self.label = self.myfont.render("Subgoal: %i" % (self.wireframes['exc'].p.subgoal), 1, (200, 200, 200))
                self.screen.blit(self.label, (20, 80))

            ## Action Blending Values
            self.label = self.myfont.render("Operator: %s" % ['%.3f' % elem for elem in self.wireframes['exc'].js.tolist()], 1, (200, 200, 200))
            self.screen.blit(self.label, (20, 110))
            if self.wireframes['exc'].params['blending'] != 'off':
                self.label = self.myfont.render("Control: %s" % ['%.3f' % elem for elem in self.wireframes['exc'].pid.tolist()], 1, (200, 200, 200))
                self.screen.blit(self.label, (20, 140))
                self.label = self.myfont.render("Command: %s" %
                                                (['%.3f' % elem for elem in self.wireframes['exc'].command]),
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
            if self.display_edges:
                for edge in wf.edges:
                    pygame.draw.aaline(self.screen, self.edge_colour, (edge.start.x, edge.start.y), (edge.stop.x, edge.stop.y), 1)

            if self.display_nodes:
                for node in wf.nodes:
                    pygame.draw.circle(self.screen, self.node_colour, (int(node.x), int(node.y)), self.node_radius, 0)

    def translate_all(self, axis, d):
        """ Translate all wireframes along a given axis by d units. """

        for wf in self.wireframes.itervalues():
            wf.translate(axis, d)

    def scale_all(self, scale):
        """ Scale all wireframes by a given scale, centred on the centre of the screen. """

        centre_x = self.width/2
        centre_y = self.height/2

        for wf in self.wireframes.itervalues():
            wf.scale((centre_x, centre_y), scale)

    def rotate_all(self, axis, theta):
        """ Rotate all wireframe about the origin, along a given axis by a given angle. """

        rotate_function = 'rotate' + axis
        centre = (0, 0, 0)

        for wf in self.wireframes.itervalues():
            # centre = wireframe.findCentre()
            getattr(wf, rotate_function)(centre, theta)


if __name__ == '__main__':
    params = parse_argv(argv, params)

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
    exc = simtools.ExcWireframe(js1, js2, order, params=params)
    pv.add_wireframe('exc', exc)

    # Truck wireframe
    truck = simtools.Wireframe()
    truck.addNodes([(x, y, z) for x in (37, 67.5) for y in (-9, 9) for z in (0, 15)])
    truck.addEdges([(n, n+1) for n in [0, 2, 4, 6]] + [(n, n+4) for n in range(4)] + [(n, n+2) for n in [0, 1, 4, 5]])
    pv.add_wireframe('truck', truck)

    # Pile wireframe
    pile = simtools.Wireframe()
    pile.addNodes([(x, 38, 0) for x in (2, 42)] + [(x, 78, 0) for x in (42, 2)] + [(22, 58, 30)])
    pile.addEdges([(n, 4) for n in range(4)] + [(n, (n + 1) % 4) for n in range(4)])
    pv.add_wireframe('pile', pile)

    # Subgoal Nodes
    subgoals = simtools.Wireframe()
    subgoals.addNodes([simtools.forward_kin(mean) for mean in exc.p.means])
    pv.add_wireframe('subgoals', subgoals)

    # Prepare orientation of scene HAS TO MATCH EXC UPDATE FUNC
    pv.rotate_all('X', 1.57)
    pv.rotate_all('Y', 0.7853)
    pv.rotate_all('X', 2.8)
    pv.translate_all('x', 400)
    pv.translate_all('y', 300)
    pv.scale_all(6)
    pv.translate_all('y', 200)

    pv.run()

    # Shut her down.
    if pv.wireframes['exc'].params['logging']:
        pv.wireframes['exc'].dl.close()

    # # Save dynamic subgoal locations to pickle
    # with open('pickles/' + name_date_time_ext('dyn_sgs'), 'wb') as sg_save:
    #     pickle.dump(pv.wireframes['exc'].p.sg_list, sg_save)
