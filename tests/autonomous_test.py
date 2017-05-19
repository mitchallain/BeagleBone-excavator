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
import time
import json
import logging
# import pdb
from itertools import cycle


def load_task():
    ''' Load an array of end points from serial file '''
    with open('task.json', 'rb') as jf:
        task = json.loads(jf.read())
    return task


def init_controllers():
    boom_PI = PID(1.25, 0.1, 0, 0, 0, 10, -10)
    stick_PI = PID(1.25, 0.1, 0, 0, 0, 10, -10)
    bucket_PI = PID(1.25, 0.1, 0, 0, 0, 10, -10)
    swing_PI = PID(1.75, 0.1, 0, 0, 0, 10, -10)
    return [boom_PI, stick_PI, bucket_PI, swing_PI]


def main():
    # LOG
    # logging.basicConfig(filename='auto.log', level=logging.DEBUG)
    
    actuators, measurements = exc.exc_setup()
    
    filename = raw_input('Name the output file (end with .csv) or press return to disable data logging: ')
    if filename == '':
        print('No data storage selected')
    else:
        print('Writing headers to: ' + filename)
        data = exc.DataLogger(6, filename)
    
    task = load_task()
    acts = ['bm', 'sk', 'bk', 'sw']
    with open('prop.json', 'rb') as f:
        prop = json.loads(f.read())

    for m in measurements:
        m.update_measurement()

    controllers = init_controllers()
    start = time.time()

    try:
        for pt in cycle([task[str(i)] for i in range(6)]):
            # pdb.set_trace()
            dur = traj.duration([m.value for m in measurements],
                                pt, [prop[a]['vmax'] for a in acts],
                                [prop[a]['amax'] for a in acts])
            coeff = traj.quintic_coeff(dur, [m.value for m in measurements], pt)[::-1]
    
            end = np.polyval(coeff, dur)
            end_error = np.array([m.value for m in measurements]) - end
            track_error = np.zeros(4)
            ref = np.array([m.value for m in measurements])
            
            for m, c in zip(measurements, controllers):
                c.setPoint(m.value)
                c.setIntegrator(0)
                c.setDerivator(0)
    
            raw_input('Next movement to point: [%.2f, %.2f, %.2f, %.2f]\n'
                      'Duration: %.2f sec\n'
                      'Pausing...' % (tuple(pt) + (dur,)))
            traj_start = time.time()
            # pdb.set_trace()
    
            while (np.abs(end_error) > np.array([0.5, 0.5, 0.5, 0.07])).any():
                loop_start = time.time()
                t = loop_start - traj_start
    
                for m in measurements:
                    m.update_measurement()
    
                state = np.array([m.value for m in measurements])
    
                error = ref - state
                end_error = end - state
    
                for i, (a, c) in enumerate(zip(actuators, controllers)):
                    if (loop_start - traj_start) < dur:  # not the most elegant way to do this
                        ref[i] = np.polyval(coeff[:, i], t)
                        c.setPoint(ref[i])
                    a.command = c.update(state[i])
    
                exc.safe_action(measurements, actuators)
    
                # Data logging mode 6 (autonomous2.0)
                try:
                    data.log([loop_start - start, loop_start - traj_start] +                 # Run-time clock
                             [m.value for m in measurements] +      # BM, ST, BK, SW Measurements
                             [a.duty_set for a in actuators] +      # BM, ST, BK, SW Duty Cycle Cmd
                             [c.getError() for c in controllers] +  # BM, ST, BK, SW Error
                             [c.set_point for c in controllers])
                except NameError:
                    pass
                except:
                    raise
                
                while time.time() - loop_start < 0.05:
                    pass
    
            for a in actuators:  # Reset all duty cycles between trajectories
                a.duty_set = a.duty_mid
                a.update_servo()

    except KeyboardInterrupt:
        print '\nHoming...'
        exc.homing(actuators, measurements, controllers, [10, 10, 2, 0], 
                  [0.3, 0.3, 0.3, 0.05], 10)
    except:
        raise
    finally:
        print '\nClosing PWM signals...'
        for a in actuators:
            a.duty_set = a.duty_mid
            a.update_servo()
            time.sleep(0.2)
            a.close_servo()
        if 'data' in locals():
            data.close()


if __name__ == '__main__':
    main()
