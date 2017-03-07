#! /usr/bin/env python

##########################################################################################
# write_joystick.py
#
# Writes a single joystick to a csv file
#
# NOTE:
#
# Created: September 14, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

import pygame
import time

pygame.init()
pygame.joystick.init()
tm_right = pygame.joystick.Joystick(0)
tm_right.init()

f_name = raw_input('Enter a filename: ')
f = open('data/'+f_name, 'w')

st = time.time()

try:
    while True:
        pygame.event.pump()
        pos = [str(tm_right.get_axis(i)) for i in range(2)]
        print(pos)
        f.write(str(time.time()-st)+',')
        for elem in pos:
            f.write(elem+',')
        f.write('\n')
        time.sleep(0.1)
        # print(pos, type(pos), pos[1], pos[1][1])
except KeyboardInterrupt:
    print 'Quitting'
    f.close()
