#! /usr/bin/env python

##########################################################################################
# printZ.py
#
# Poll events from spacenavigator 3D mouse and print z rotation torque to standard output.
#
# NOTE:  written as a test for servo control via 3D mouse
#
# Created: 07/11/16
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

from spnav import *

if __name__ == '__main__':
    spnav_open()
    try:
        while True:
            event = spnav_poll_event()
            if event is not None:
                print event.rotation[1]
    except KeyboardInterrupt:
        print '\nQuitting...'
    finally:
        spnav_close()
