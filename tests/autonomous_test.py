#! /usr/bin/env python

##########################################################################################
# autonomous_test.py
#
# Rewrite of autonomous.py for control tests
#
# Note: May experiment with new PID module
#
# Created: May 17, 2017
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
# To Do:
#   * DEBUG
#
##########################################################################################

import excavator as exc
import trajectories as traj
import numpy as np
from PID import PID
import pickle
import time


def load_end_pts():
    ''' Load an array of end points from serial file '''
    with open('autonomous_task.pkl', 'rb') as pkl:
        task = pickle.load(pkl)
    return task


def load_logger():
    ''' Load datalogger or set none to data '''
    filename = raw_input('Name the output file (end with .csv) or press return to disable data logging: ')
    if filename == '':
        print('No data storage selected')
    else:
        print('Writing headers to: ' + filename)
        data = exc.DataLogger(2, filename)
        return data


def init_controllers():
    boom_PI = PID(1.25, 0.1, 0, 0, 0, 10, -10)
    stick_PI = PID(1.25, 0.1, 0, 0, 0, 10, -10)
    bucket_PI = PID(1.25, 0.1, 0, 0, 0, 10, -10)
    swing_PI = PID(1.75, 0.1, 0, 0, 0, 10, -10)
    return [boom_PI, stick_PI, bucket_PI, swing_PI]


def main():
    actuators, measurements = exc.exc_setup()
    data = load_logger()
    task = load_end_pts()
    with open('prop.json', 'rb') as f:
        prop = json.loads(f)

    for m in measurements:
        m.update_meaurement()

    controllers = init_controllers()

    try:
        for pt in task:
            dur = traj.duration([m.value for m in measurements],
                                pt, [a_prop['vmax'] for a in prop.values()],
                                [a_prop['amax'] for a in prop.values()])
            coeff = traj.quintic_coeff(dur, [m.value for m in measurements], pt)

            end = np.poly.polyval(coeff, dur)
            end_error = np.array([m.value for m in measurements]) - end
            track_error = np.zeros(4)

            raw_input('Next movement duration: %.2f sec\nPausing...' % dur)
            traj_start = time.time()

            while (np.abs(end_error) > np.array([])).any():
                loop_start = time.time()
                t = loop_start - traj_start

                for m in measurements:
                    m.update_measurement()

                state = np.array([m.value for m in measurements])

                # check this polyval function
                ref = np.poly.polyval(t, coeff)
                error = state - ref

                for i, (a, c) in enumerate(zip(actuators, controllers)):
                    if loop_start < dur:
                        c.setPoint(np.poly.polyval(t, coeff[i]))  # check this
                    a.command = c.update(state[i])

                exc.safe_action(measurements, actuators)

                # Data logging mode 6 (autonomous2.0)
                try:
                    data.log([loop_start - start] +                 # Run-time clock
                             [m.value for m in measurements] +      # BM, ST, BK, SW Measurements
                             [a.duty_set for a in actuators] +      # BM, ST, BK, SW Duty Cycle Cmd
                             [c.getError() for c in controllers] +  # BM, ST, BK, SW Error
                             [c.set_point for c in controllers])
                except NameError:
                    pass

            for a in actuators:  # Reset all duty cycles between trajectories
                a.duty_set = a.duty_mid
                a.update_servo()



                



