#! /usr/bin/env python

##########################################################################################
# model_gen.py
#
# Use this code to determine endpoints in task, and create subgoal model
#
# NOTE: Start with position above pile, after swinging from dump
#
# Created: October 19, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

from excavator import *
import pickle

f = open('sg_model.pckl', 'wb')
points = []
measurements = measurement_setup()

try:
    while True:
        for m in measurements:
            m.update_measurement()
            print(m.measure_type + ': ' + str(m.value))

        points.append([m.value for m in measurements])
        print points
        raw_input('Press Enter to advance...')
except KeyboardInterrupt:
    sg_swing_to_pile = {'subgoal': 1,               # SWING OVER TO PILE
                        'it': [3, -0.5],            # Joystick index 3 (swing) move left
                        'subgoal_pos': points[0],   # Over the pile
                        'npt': [3, 3, 3, 0.2],      # Terminate when swing close to the pile
                        'onpt': []}

    sg_pre_dig = {'subgoal': 2,                     # IN PILE BEFORE SCOOP
                  'subgoal_pos': points[1],         # Subgoal position: in pile prepared for dig
                  'it': [0, -0.5],                  # Input Trigger: boom down
                  'npt': [3, 3, 3, 0.2],            # NPT: bucket uncurled
                  'onpt': []}

    sg_post_dig = {'subgoal': 3,                    # IN PILE AFTER SCOOP
                   'subgoal_pos': points[2],        # Subgoal position: uncurled over truck
                   'it': [1, 0.5],                  # Input Trigger: stick out
                   'npt': [3, 3, 3, 0.2],           # NPT: bucket uncurled
                   'onpt': []}

    sg_lift_to_pile = {'subgoal': 4,                # LIFT ABOVE PILE
                       'subgoal_pos': points[3],    # Subgoal position: uncurled over truck
                       'it': [0, 0, -0.5, 0],       # Input Trigger: bucket uncurl cmd
                       'npt': [3, 3, 3, 0.2],       # NPT: bucket uncurled
                       'onpt': []}

    sg_over_truck = {'subgoal': 5,                  # OVER TRUCK UNDUMPED
                     'subgoal_pos': points[4],      # Subgoal position: uncurled over truck
                     'it': [0, 0, -0.5, 0],         # Input Trigger: bucket uncurl cmd
                     'npt': [3, 3, 3, 0.2],         # NPT: bucket uncurled
                     'onpt': []}

    sg_dump = {'subgoal': 6,                        # OVER TRUCK DUMPED
               'subgoal_pos': points[5],            # Subgoal position: uncurled over truck
               'it': [0, 0, -0.5, 0],               # Input Trigger: bucket uncurl cmd
               'npt': [3, 3, 3, 0.2],               # NPT: bucket uncurled
               'onpt': []}
    sg_model = [sg_swing_to_pile, sg_pre_dig, sg_post_dig, sg_lift_to_pile, sg_over_truck, sg_dump]

    pickle.dump(sg_model, f)
finally:
    f.close()
