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


def load_logger():
    # Initialize DataLogger class with mode 2 for autonomy
    filename = raw_input('Name the output file (end with .csv) or press return to disable data logging: ')
    if filename == '':
        print('No data storage selected')
    else:
        print('Writing headers to: ' + filename)
        data = exc.DataLogger(2, filename)
        return data


def main():
    data = load_logger()

    # Init PWM classes and measurement classes, zero encoder
    actuators, measurements = exc.exc_setup()

    # Desired movement sequence (BM up, SK out, BK in, SW ccw, SW cw,
    #                            BK out, SK in, BM down)
    seq = (0, 1, 2, 3, 3, 2, 1, 0)
    duties = []  # list of np arrays of duty commands

    for i, a in enumerate(actuators):  # build up test array
        pos = np.linspace(a.duty_mid, a.duty_max, 26)
        neg = np.linspace(a.duty_mid, a.duty_min, 26)
        c = np.zeros(52)
        c[0::2] = pos
        c[1::2] = neg
        duties.append(c)

    j = [0]*4
    flag = 1  # flag marks the beginning of a new actuator movement
    start = time.time()

    for m in measurements:
        m.update_measurement()

    try:
        while j[0] < 52:  # BM less than 52 times
            for i in seq:
                seq_start = time.time()
                while exc.check_in_bounds(measurements, actuators) and ((time.time() - start) < 10):
                    loop_start = time.time()

                    j[i] += 1
                    actuators[i].duty_set = duties[i][j]
                    print('%s Duty Cycle: %.3f' % (actuators[i].actuator_name, duties[i][j]))
                    actuators[i].update_servo()

                    try:
                        data.log([time.time() - start, time.time() - seq_start] +  # Run-time clock
                                 [a.duty_set for a in actuators] +      # BM, ST, BK, SW Duty Cycle Cmd
                                 [b.value for b in measurements] +      # BM, ST, BK, SW Measurements
                                 [flag, i])
                    except NameError:
                        pass

                    flag = 0

                    while (time.time() - loop_start) < 0.025:
                        pass

                for a in actuators:
                    a.duty_set = a.duty_mid
                    a.update_servo()

                flag = 1

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
