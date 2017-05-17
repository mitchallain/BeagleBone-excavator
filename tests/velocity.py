#! /usr/bin/env python

##########################################################################################
# velocity.py
#
# Test the steady-state velocity of the actuators for each PWM duty cycle
#
# NOTE:
#
# Created: May 16, 2017
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

import excavator as exc
import numpy as np
import time
# import pdb


def load_logger():
    # Initialize DataLogger class with mode 2 for autonomy
    filename = raw_input('Name the output file (end with .csv) or press return to disable data logging: ')
    if filename == '':
        print('No data storage selected')
    else:
        print('Writing headers to: ' + filename)
        data = exc.DataLogger(5, filename)
        return data


def main():
    data = load_logger()

    # Init PWM classes and measurement classes, zero encoder
    actuators, measurements = exc.exc_setup()

    # Desired movement sequence (BM up, SK out, BK in, SW ccw, SW cw,
    #                            BK out, SK in, BM down)
    seq = (0, 1, 2, 2, 1, 0)
    duties = []  # list of np arrays of duty commands

    for i, a in enumerate(actuators):  # build up test array
        pos = np.linspace(a.duty_mid + 1.1, a.duty_max, 15)
        neg = np.linspace(a.duty_mid - 1.1, a.duty_min, 15)
        c = np.zeros(30)
        c[0::2] = pos
        c[1::2] = neg
        duties.append(c)

    j = [0]*4
    flag = 1  # flag marks the beginning of a new actuator movement
    start = time.time()

    for m in measurements:
        m.update_measurement()

    try:
        while j[0] < 30:  # BM less than 52 times
            for i in seq:
                seq_start = time.time()

                actuators[i].duty_set = duties[i][j[i]]
                actuators[i].update_servo()

                while (exc.check_in_bounds([measurements[i]], [actuators[i]]) or (time.time() - seq_start < 3)) and (time.time() - seq_start < 5):
                    loop_start = time.time()
                    # print(exc.check_in_bounds([measurements[i]], [actuators[i]]))
                    
                    # pdb.set_trace()
                    print('%s Duty Cycle: %.3f, j: %s, Clock: %.2f' % (actuators[i].actuator_name, duties[i][j[i]], j, time.time() - start))
                    
                    for m in measurements:
                        m.update_measurement()

                    try:
                        data.log([time.time() - start, time.time() - seq_start] +  # Run-time clock
                                 [a.duty_set for a in actuators] +      # BM, ST, BK, SW Duty Cycle Cmd
                                 [m.value for m in measurements] +      # BM, ST, BK, SW Measurements
                                 [flag, i])
                    except NameError:
                        pass
                    except AttributeError:
                        pass

                    flag = 0

                    while (time.time() - loop_start) < 0.025:
                        pass

                j[i] += 1
                for a in actuators:
                    a.duty_set = a.duty_mid
                    a.update_servo()

                flag = 1
    # print(j, i)

    except KeyboardInterrupt:
        print '\nQuitting'
    finally:
        print '\nClosing PWM signals...'
        for a in actuators:
            a.duty_set = a.duty_mid
            a.update_servo()
        time.sleep(1)
        for a in actuators:
            a.close_servo()
        if 'data' in locals():
            data.file.close()


if __name__ == '__main__':
    main()
