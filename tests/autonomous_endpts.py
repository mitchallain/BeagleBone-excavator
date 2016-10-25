#! /usr/bin/env python

##########################################################################################
# autonomous_endpts.py
#
# Use this code to determine endpoints in task, and create autonomous trajectories
#
# NOTE: Start with position above pile, after swinging from dump, 
#       use autonomous_trajectories.ipynb to create traj's
#
# Created: October 24, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

from excavator import *
import pickle

pts = []
measurements = measurement_setup()

try:
    while True:
        raw_input('Press Enter to advance...')
        for m in measurements:
            m.update_measurement()
            print(m.measure_type + ': ' + str(m.value))
        pts.append([m.value for m in measurements])
        print pts[-1]
except KeyboardInterrupt:
    f = open('end_pts.pkl', 'wb')
    pickle.dump(pts, f)
finally:
    f.close()
    print('Data pickled and file closed.')
