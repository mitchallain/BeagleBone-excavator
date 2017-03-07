#! /usr/bin/env python

##########################################################################################
# client_synchronous.py
#
# A client to communicate with server_synchronous.py
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
import sys
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
except socket.error, msg:
    print('Failed to create socket. Error code: ' + str(msg[0]) + ' , Error message: ' + msg[1])
    sys.exit()
