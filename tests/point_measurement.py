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
#   * May 11, 2017 - namespace fix
#   *
#
##########################################################################################

import excavator as exc
import pdb

points = []
measurements = exc.measurement_setup()

try:
    while True:
        for m in measurements:
            m.update_measurement()
        # pdb.set_trace()
        print('%8s %8s %8s %8s\n%8.3f %8.3f %8.3f %8.3f' % 
              tuple([m.measure_type for m in measurements] + [m.value for m in measurements]))
    
        points.append([m.value for m in measurements])
        raw_input('Press Enter to advance...')
finally:
    print points