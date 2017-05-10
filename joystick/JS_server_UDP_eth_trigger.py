#! /usr/bin/env python

##########################################################################################
# JS_server_UDP_eth_trigger.py
#
# Special usage of JS_server for demonstrating subgoal locations
#
# NOTE: sends [BM, SK, BK, SW, get_button(0)], see UDP_client_test.py for parsing example
#
# Created: March 09, 2017
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

import socket
import pygame
import time

UDP_IP = '192.168.10.2'
UDP_PORT = 9999

print "UDP IP: ", UDP_IP
print "UDP target port: ", UDP_PORT

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print('Initializing pygame module and joysticks...\n')
pygame.init()
pygame.joystick.init()
tm_1 = pygame.joystick.Joystick(1)
tm_1.init()
tm_0 = pygame.joystick.Joystick(0)
tm_0.init()

print('Squeeze trigger on right joystick...')
while True:
    pygame.event.pump()
    if tm_0.get_button(0) == 1:
        order = [1, 3, 0, 2]
        TM_0_RIGHT = True
        break
    elif tm_1.get_button(0) == 1:
        order = [3, 1, 2, 0]
        TM_0_RIGHT = False
        break
print('Right joystick detected.')

# print("Connected to:" + self.client_address[0])
# # Likewise, self.wfile is a file-like object used to write back
# # to the client
trigger_delay = time.time()

while True:
    pygame.event.pump()
    js = [tm_0.get_axis(0), tm_0.get_axis(1), tm_1.get_axis(0), tm_1.get_axis(1)]

    if TM_0_RIGHT:
        trigger_state = tm_0.get_button(0)
    else:
        trigger_state = tm_1.get_button(0)

    # Fix this hack
    if (time.time() - trigger_delay > 1) and (trigger_state == 1):
        trigger_delay = time.time()
    else:
        trigger_state = 0

    # Send: [BM, SK, BK, SW]
    sock.sendto(str([js[i] for i in order] + [trigger_state]), (UDP_IP, UDP_PORT))

    time.sleep(0.05)
