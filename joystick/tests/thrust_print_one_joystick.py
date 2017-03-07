#! /usr/bin/env python

##########################################################################################
# thrustmaster_print_loop.py
#
# Prints the axis positions for the Thrustmaster T16000M using pygame joystick stuff
#
# NOTE: Will extend for both thrustmasters on Linux system (BBB)
#
# Created: August 11, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   * added support for right AND left joysticks - August 19, 2016
#
##########################################################################################

import pygame

pygame.init()
pygame.joystick.init()
tm_right = pygame.joystick.Joystick(0)
tm_right.init()
# tm_left = pygame.joystick.Joystick(1)
# tm_left.init()

while True:
    pygame.event.pump()
    pos = [tm_right.get_axis(1), tm_right.get_axis(2), 0, 0]
    button = tm_right.get_button(0)
    print(pos, button)
    # print(pos, type(pos), pos[1], pos[1][1])
