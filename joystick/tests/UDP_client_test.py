#! /usr/bin/env python

##########################################################################################
# UDP_client_test.py
#
# Got to get it working first
#
# NOTE:
#
# Created: October 19, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

import socket


UDP_IP = ''
UDP_PORT = 9999

# Create a socket (SOCK_DGRAM means a UDP socket)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    # Connect to server and send data
    sock.bind((UDP_IP, UDP_PORT))

    while True:
        # Receive data from the server and shut down
        received = sock.recv(4096)  # Buffer size 4096
        # Parse that junk
        received = received.translate(None, "[( )]").split(',')
        received = [float(i) for i in received]
        print(received)
finally:
    sock.close()
