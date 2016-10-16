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
# TODO: - Swing error scaling
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
#   * October 15, 2016 - encoder integration
#   *
#
##########################################################################################

from excavator import *
# import socket
import time
import numpy.polynomial.polynomial as poly
from PID import PID


# Create some trajectories, trajectory_# = [time, [poly coeff's starting with lowest power for BM], [SK poly], [BK poly], [SW poly]]
trajectory_1 = [[0, [8.185, 0, 0, 0, 0]], [2.997, [4.63, 0, 0, 1.1889, -0.5951, 0.0794]], [2.997, [3.5065, 0, 0, 2.3539, -1.1781, 0.1572]], [0, [0, 0, 0, 0]]]
# trajectory_2 = 
task = [trajectory_1]

# Initialize PWM/servo classes and measurement classes, note: this zeros the encoder
temp = exc_setup()
actuators = temp[0]
measurements = temp[1]

# Initialize DataLogger class with mode 2 for autonomy
filename = raw_input('Name the output file (end with .csv) or press return to disable data logging: ')
if filename == '':
    print('No data storage selected')
else:
    print('Writing headers to: ' + filename)
    data = DataLogger(2, filename)

# Zero the encoder again
# measurements[3].encoder.zero()

# Or preset the value
measurements[3].encoder.setPosition(1201)

# Set duty cycles to middle, and initialize Measurement classes
for a in actuators:
    a.update_servo()

for b in measurements:
    b.update_measurement()

# PI Controllers for each actuator
boom_PI = PID(1.25, 0.1, 0, 0, 0, 10, -10)
stick_PI = PID(1.25, 0.1, 0, 0, 0, 10, -10)
bucket_PI = PID(1.25, 0.1, 0, 0, 0, 10, -10)
swing_PI = PID(1.75, 0.1, 0, 0, 0, 10, -10)
controllers = [boom_PI, stick_PI, bucket_PI, swing_PI]

# BEGIN HOMING, Homing routine to set point
home = [8.185, 4.630, 3.5065, 1.2083]

for i in range(4):
    controllers[i].setPoint(home[i])
    controllers[i].update(measurements[i].value)
tempstart = time.time()

try:
    while np.linalg.norm([controllers[i].getError() for i in range(2)]+[controllers[3].getError()*30]) > 1:    # 1 cm radius ball about endpoint
        # Measurement
        for b in measurements:
            b.update_measurement()
            print(b.value)
    
        for i in range(4):
                # Update actuators with control action
                actuators[i].duty_set = controllers[i].update(measurements[i].value) + actuators[i].duty_mid

        # Update PWM, saturation implemented in Servo class
        for a in actuators:
            a.update_servo()
            print(a.actuator_name + ': ' + str(a.duty_set))
        print('Error' + str(np.linalg.norm([controllers[i].getError() for i in range(2)]+[controllers[3].getError()*30])))

    for b in measurements:
        print(b.value)
except KeyboardInterrupt:
    print '\nClosing PWM signals...'
    for a in actuators:
        a.duty_set = a.duty_mid
        a.update_servo()
    time.sleep(1)
    for a in actuators:
        a.close_servo()
# END HOMING

start = time.time()

try:
    for trajectory in task:
        # Endpoint error norm triggers trajectory change
        endpoint = [0, 0, 0, 0]
        endpoint_error = [0, 0, 0, 0]
        
        # Initialize integrator and derivator to zero
        for c in controllers:
            c.setIntegrator(0)
            c.setDerivator(0)

        # Determine endpoints for each *active* actuator and an endpoint error vector (actuator space)
        for i in range(4):
            if trajectory[i][0] == 0:
                endpoint_error[i] = 0
                endpoint[i] = 0
            else:
                endpoint[i] = poly.polyval(trajectory[i][0], trajectory[i][1])
                endpoint_error[i] = measurements[i].value - endpoint[i]
                endtime = trajectory[i][0]
        endpoint_error[3] = endpoint_error[3]*30  # scale swing error, [0, 1.571] to [0, 45], i.e., 2 degrees are penalized same as 1 cm on actuator

        # Time markers
        traj_start = time.time()
        loop_start = time.time()

        while np.linalg.norm(endpoint_error) > 1 or (loop_start-traj_start) < endtime:    # 1 cm radius ball about endpoint
            loop_start = time.time()

            # Measurement
            for b in measurements:
                b.update_measurement()

            # Set point update and PI output
            for i in range(4):
                if trajectory[i][0] != 0:
                    # Change set point only if still within valid trajectory window
                    if (time.time() - traj_start) < trajectory[i][0]:
                        controllers[i].setPoint(poly.polyval((time.time()-traj_start), trajectory[i][1]))

                    # Update actuators with control action and endpoint error
                    # actuators[i].duty_set = actuators[i].duty_span*(controllers[i].update(measurements[i].value) + 1)/2 + actuators[i].duty_mid
                    actuators[i].duty_set = controllers[i].update(measurements[i].value) + actuators[i].duty_mid
                    print(controllers[i].P_value + controllers[i].I_value + controllers[i].D_value)
                    endpoint_error[i] = measurements[i].value - endpoint[i]
            

            # Update PWM, saturation implemented in Servo class
            for a in actuators:
                a.update_servo()
                print(a.actuator_name + ': ' + str(a.duty_set))

    # Data logging mode 3 (autonomous)
    try:
        data.log(loop_start - start,                    # Run-time clock
                 [b.value for b in measurements] +      # BM, ST, BK, SW Measurements
                 [a.duty_set for a in actuators] +      # BM, ST, BK, SW Duty Cycle Cmd
                 [c.getError() for c in controllers])   # BM, ST, BK, SW Error
    except NameError:
        pass


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
