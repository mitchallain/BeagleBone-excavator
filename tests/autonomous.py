#! /usr/bin/env python

##########################################################################################
# autonomous.py
#
# Derived from manual.py
# Autonomous operation of the excavator. Records data and timestamps.
#
# NOTE: SERVO CLASS, INTERPOLATE FUNCTION, AND MISC DEPENDENCIES IMPORTED FROM EXCAVATOR.PY
#       function task_description imports output from shyams work and creates list of list of polys
#       controllers act on error until within ball of endpoints in actuator space
#
#
# TODO: - Add integral action
#       - Test and validate, timeit()
#       - Fix time on BBB for datastamping
#
# Created: September 27, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   * September 28, 2016 - changed to autonomous.py, implemented poly func
#   * October 11, 2016 - point to point actuator space trajectories in each
#   * October 11, 2016 - going to switch dependecies to excavator
#   *
#   *
#
##########################################################################################

from excavator import *
# import socket
import time
import numpy.polynomial.polynomial as poly
from PID import PID

# def task_description(f):
#     # parse csv, or whatever filetype
#     # create list of lists of polys
#     # each list of poly's describes one pt to pt trajectory
#     # controller acts on error until

# def homing(PWM_funcs, measure):
#     home_pos = [97.6, 65.1, 60.4, 1.2083]
#     while

# def saturate(low, high, value):
#     return max(low, min(high, value))


# Measurement initialization
ADC.setup()

# Initialize PWM/servo classes and measurement classes
temp = excavator.setup()
actuators = temp[0]
measurements = temp[1]

# Create some trajectories
task = [[4.5, [74.7, 8, 6, -2]]]

# Create a socket (SOCK_STREAM means a TCP socket)
# sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Data saving interface
f_name = raw_input('Name the output file (end with .csv) or press return for no data: ')
if f_name == '':
    print('No data storage selected.')
elif f_name[-4:] != '.csv':
    print("\nPlease use '.csv' extension")
    quit()
else:
    try:
        f = open('data/'+f_name, 'w')
    except IOError:
        print('IOError')

    f.write('Time,Boom Error,Stick Error,Bucket Error,Swing Error,Boom Cmd,Stick Cmd,Bucket Cmd,Swing Cmd,Boom Ms,Stick Ms,Bucket Ms,Swing Ms\n')

# # Initialize trajectory (coeff beginning with lowest order)
# coeff = (74.7, 8, 6, -2)

# Set duty cycles to middle, and initialize Measurement classes
for a in actuators:
    a.update_servo()

for b in measurements:
    b.update_measurement()

# PI Controllers for each actuator
boom_PI = PID(0.5, 1, 0, 0, 10, -10)
stick_PI = PID(0, 0, 0, 0, 10, -10)
bucket_PI = PID(0, 0, 0, 0, 10, -10)
swing_PI = PID(0, 0, 0, 0, 10, -10)
controllers = [boom_PI, stick_PI, bucket_PI, swing_PI]

start = time.time()
endpoint = [0, 0, 0, 0]
endpoint_error = [0, 0, 0, 0]

try:
    for trajectory in task:
        for i in range(4):
            endpoint[i] = poly.polyval(trajectory[i][0], trajectory[i][1])
            endpoint_error[i] = measurements[i].value - endpoint[i]

        traj_start = time.time()

        while np.linalg.norm(endpoint_error) > 1:    # 1 cm radius ball about endpoint
            loopstart = time.time()

            # Measurement
            for b in measurements:
                b.update_measurement()

            # Set point update and PI output
            for i in range(4):

                # Change set point only if still within valid trajectory window
                if (time.time() - traj_start) < trajectory[i][0]:
                    controllers[i].setPoint(poly.polyval((time.time()-traj_start), trajectory[i][1]))

                # Update actuators with control action and endpoint error
                actuators[i].duty_set = actuators[i].duty_span*(controllers[i].update(measurements[i].value) + 0.5) + actuators[i].duty_mid
                endpoint_error[i] = measurements[i].value - endpoint[i]

            # Update PWM, saturation implemented in Servo class
            for a in actuators:
                # a.update_servo()
                print(a.actuator_name + ': ', a.duty_set, '\n')

        # # Error signal
        # ref = poly.polyval((time.time()-start), coeff)
        # e = ref - boom_ms

        # # Generate control signals
        # boom_duty_cycle = boom.duty_span*((e/10) + 1)/(2) + boom.duty_min
        # boom_duty_cycle_input = saturate(boom_duty_cycle, 4.939, 10.01)
        # PWM.set_duty_cycle(boom.servo_pin, boom_duty_cycle_input)
        # print(e, boom_duty_cycle_input)

            # Data Logging
            try:
                f.write(str(loopstart - start) + ',' +                # Time
                        str(controllers[0].getError()) + ',' +        # Boom e
                        str(controllers[1].getError()) + ',' +        # Stick e
                        str(controllers[2].getError()) + ',' +        # Bucket e
                        str(controllers[3].getError()) + ',' +        # Swing e
                        str(actuators[0].duty_set) + ',' +            # Boom Cmd
                        str(actuators[1].duty_set) + ',' +            # Stick Cmd
                        str(actuators[2].duty_set) + ',' +            # Bucket Cmd
                        str(actuators[3].duty_set) + ',' +            # Swing Cmd
                        str(measurements[0].value) + ',' +            # Boom Ms
                        str(measurements[1].value) + ',' +            # Stick Ms
                        str(measurements[2].value) + '\n')            # Bucket Ms
            except NameError:
                pass

except KeyboardInterrupt:
    print '\nQuitting'
finally:
    print '\nClosing PWM signals...'
    for a in actuator:
        a.duty_set = a.duty_mid
        a.update_servo()
    time.sleep(1)
    for a in actuator:
        a.close_servo()
