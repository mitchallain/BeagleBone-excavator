#! /usr/bin/env python

##########################################################################################
# client_UDP_with_trigger.py
#
# server client for local testing on MBP with triggering
#
# NOTE:
#
# Created: August 18, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

import socket
# import pdb
import time


def parse_joystick_with_trigger(received, received_parsed):
    '''Parse joystick data from server_02.py, and convert to float'''
    deadzone = 0.2
    toggle_invert = [1, 1, -1, -1]  # Invert [BM, SK, BK, SW] joystick
    try:
        received = received.translate(None, "[( )]").split(',')
        for axis in xrange(len(received) - 1):
            if (float(received[axis]) > deadzone) or (float(received[axis]) < -deadzone):
                received_parsed[axis] = float(received[axis])*toggle_invert[axis]
            else:
                received_parsed[axis] = 0
        return received_parsed[0:4], int(received[4])
    except ValueError:
        print '\nValue Error'
        raise ValueError

HOST, PORT = '169.254.18.40', 9999

# Create a socket (SOCK_STREAM means a TCP socket)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

received_parsed = [0, 0, 0, 0]

try:
    # Connect to server and send data
    sock.bind((HOST, PORT))
    while True:
        start = time.time()
        print parse_joystick_with_trigger(sock.recv(4096), received_parsed)

        # Soft timing
        while (time.time() - start) < 0.05:
            pass
finally:
    sock.close()
