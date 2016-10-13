#! /usr/bin/env python

##########################################################################################
# point_measurement.py
#
# Use this code to determine endpoints in task
#
# NOTE: Code will print measurements each time return is hit
#
# Created: October 12, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

from excavator import *


measurements = measurement_setup()

while True:
    for m in measurements:
        m.update_measurement()
        print(m.measure_type + ': ' + str(m.value))
    raw_input('Press Enter to advance...')
