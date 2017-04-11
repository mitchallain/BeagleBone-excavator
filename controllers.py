#! /usr/bin/env python

##########################################################################################
# controllers.py
#
# Store different controller configurations
#
# NOTE: use date as naming convention
#
# Created: April 06, 2017
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

from PID import PID

## ORDER OF ARGUMENTS
# PID(Kp, Ki, Kd, Id, Ii, Imax, Imin)
# [BM, SK, BK, SW]


## CONTROLLERS

# Controller from IFAC tests
controllers_ifac = [PID(1, 0.1, 0, 0, 0, 4, -4),
                    PID(1.5, 0.1, 0, 0, 0, 4, -4),
                    PID(1, 0.1, 0, 0, 0, 4, -4),
                    PID(10, 0.1, 0, 0, 0, 4, -4)]

# Controllers from autonomous
controllers_auto = [PID(1.25, 0.1, 0, 0, 0, 10, -10),
                    PID(1.25, 0.1, 0, 0, 0, 10, -10),
                    PID(1.25, 0.1, 0, 0, 0, 10, -10),
                    PID(1.75, 0.1, 0, 0, 0, 10, -10)]
