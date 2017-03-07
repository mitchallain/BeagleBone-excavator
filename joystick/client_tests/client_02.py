#! /usr/bin/env python

##########################################################################################
# client_02.py
#
# server client
#
# NOTE: I need to clean up and fix this stuff bigtime!
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


HOST, PORT = '192.168.7.1', 9999

# Create a socket (SOCK_STREAM means a TCP socket)
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    # Connect to server and send data
    sock.connect((HOST, PORT))
    while True:
        # Receive data from the server and shut down
        received = sock.recv(4096)
        # Parse that junk
        received = received.translate(None, "[( )]").split(',')
        received = [float(i) for i in received]
        print(received)
finally:
    sock.close()
