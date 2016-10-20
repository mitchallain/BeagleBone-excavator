#! /usr/bin/env python

##########################################################################################
# prediction_test.py
#
# A standalone class and attributes for prediction unit testing on MacBook (no BBB imports)
#
# NOTE: should match TriggerPrediction in excavator.py
#
# Created: October 19, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   * DEPRECATED, DO NOT EDIT COPY FROM EXCAVATOR.PY
#
##########################################################################################

# Trying out some subgoal dicts, since they do not need to have information storage and are essentially just task models
sg_pile = {'subgoal': 1,
           'it': [3, -0.5],                                  # Joystick index 3 (swing) move left
           'subgoal_pos': [6.7528, 0.9117, 9.9466, 1.4058],  # Over the pile
           'npt': [3, 3, 3, 0.2],                            # Terminate when swing close to the pile
           'onpt': []}

sg_dump = {'subgoal': 2,                                     # WILL BE 6, JUST NEED THE ITERATOR TO WORK
           'subgoal_pos': [6.7528, 0.9117, 9.9466, 0.32],    # Subgoal position: uncurled over truck
           'it': [0, 0, -0.5, 0],                               # Input Trigger: bucket uncurl cmd
           'npt': [3, 3, 3, 0.2],                            # NPT: bucket uncurled
           'onpt': []}                                       # ONPT:

sg_dig_model = [sg_pile, sg_dump]           # Listed dicts form model
del sg_pile, sg_dump                        # Clean-up namespace


class TriggerPrediction():
    '''The trigger prediction class uses task specific event triggers to determine the current subgoal

    Args:
        mode (int): 0 is off, 1 is static alpha, 2 is dynamic alpha
        model (list: dicts): list of subgoal model dicts, see example below
        alpha (float): BSC blending parameter preset for static mode

    Example arg:
        sg_model = [{'subgoal': 1,
                     'it': [3, -0.5]                            * Joystick index 3 (swing) move past halfway left
                     'subgoal_pos': [6.75, 0.91, 9.95, 1.41]    * Over the pile (actuator space coordinates)
                     'npt': [3, 3, 3, 0.2]}                     * +/- each of these values forms boundary around subgoal
                     'onpt': []},                               * Not yet implemented

                    {'subgoal': 2, ...
                    ...}]
    Attributes:
        mode (int): see above
        subgoal_model (obj):
        endpoints (list, floats): endpoints for the current task
        confidence (float): probability that current task is nominal
        blend_threshold (float): mininum confidence to initiate blending
        alpha (float): BSC blending parameter alpha
        subgoal: current triggered subgoal
        active: assistance active
        history: primitives and endpoints from recent history (window TBD)
    '''
    def __init__(self, mode, sg_model, alpha=0):
        self.mode = mode
        if self.mode == 0:  # Blending off
            self.alpha = 0
        elif self.mode == 1:  # Static alpha subgoal predictive
            self.alpha = alpha
        # elif self.mode == 2:
            #  We will see what goes here

        self.subgoal = 0            # Subgoal 0 denotes no subgoal to start
        self.active = False         # Active is bool, False means no assistance to start
        self.sg_model = sg_model

        # Important: build a list of the subgoals for iterating
        self.sg_list = [self.sg_model[i]['subgoal'] for i in range(len(self.sg_model))]

    def update_state(self, js_inputs, ms_values):
        '''Poll event triggers and update subgoal and/or active boolean.

        TODO:
            fix sloppy indexing into sg_model, subgoals index from 1

        Args:
            js_inputs (list: float): a list of js_inputs in the form [BM, SK, BK, SW]
            ms_values (list: float): listed measurement values for each actuator

        Returns:
            subgoal (int)
            active (bool)
        '''
        # Start by checking if we are in nominal task
        if self.subgoal == 0 or self.active:
            # Look for a terminating cue
            for sg in self.sg_model:
                print([abs(ms_values[i] - sg['subgoal_pos'][i]) for i in range(4)])
                # print([(ms_values[i] - sg['subgoal_pos'][i]) < sg['npt'][i] for i in range(4)])

                if [abs(ms_values[i] - sg['subgoal_pos'][i]) < sg['npt'][i] for i in range(4)] == [True]*4:
                    self.subgoal = (sg['subgoal'] % len(self.sg_list)) + 1  # i = (i % length) + 1 (some magic)
                    self.active = False

        elif not self.active:  # Assistance not active, looking for initiation
            less_than = ((js_inputs[self.sg_model[self.subgoal-1]['it'][0]] < self.sg_model[self.subgoal-1]['it'][1]))
            print less_than
            negative = ((self.sg_model[self.subgoal-1]['it'][1]) < 0)
            print negative
            if not (less_than != negative):  # If input < threshold and threshold negative, or > = threshold and threshold positive
                self.active = True

        return self.subgoal, self.active