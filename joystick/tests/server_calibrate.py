#! /usr/bin/env python

##########################################################################################
# server_calibrate.py
#
# Using SocketServer to write a server
#
# NOTE: I need to review this code and rewrite it!
#
# Created: August 18, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

import pygame
import SocketServer
import time


class MyTCPHandler(SocketServer.StreamRequestHandler):

    def handle(self):
        # self.rfile is a file-like object created by the handler;
        # we can now use e.g. readline() instead of raw recv() calls

        pygame.init()
        pygame.joystick.init()
        tm_right = pygame.joystick.Joystick(0)
        tm_right.init()
        # tm_left = pygame.joystick.Joystick(1)
        # tm_left.init()

        print("Connected to:" + self.client_address[0])
        # Likewise, self.wfile is a file-like object used to write back
        # to the client
        while True:
            pygame.event.pump()
            self.wfile.write([tm_right.get_axis(2), 0, tm_right.get_axis(0), tm_right.get_axis(1)])
            # self.wfile.write([tm_right.get_axis(i) for i in range(2)])
            time.sleep(0.1)


if __name__ == "__main__":
    HOST, PORT = '', 9999

    # Create the server, binding to localhost on port 9999
    server = SocketServer.TCPServer((HOST, PORT), MyTCPHandler)

    # Activate the server; this will keep running until you
    # interrupt the program with Ctrl-C
    server.serve_forever()
