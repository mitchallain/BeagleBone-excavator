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
import time

pygame.init()
pygame.joystick.init()
tm_right = pygame.joystick.Joystick(0)
tm_right.init()
tm_left = pygame.joystick.Joystick(1)
tm_left.init()

f_name = input('Enter a filename: ')
f = open(f_name, 'w')

st = time.time()

while True:
    pygame.event.pump()
    pos = [(str(tm_right.get_axis(i)), str(tm_left.get_axis(i))) for i in range(2)]
    print(pos)
    f.write(str(time.time()-st)+',')
    for elem in pos:
        f.write(elem+',')
    f.write('\n')
    time.sleep(0.1)
    # print(pos, type(pos), pos[1], pos[1][1])
