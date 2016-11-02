#! /usr/bin/env python

##########################################################################################
# blended.py
#
# Full impementation of blended shared control, with placeholders for future work
#
# NOTE: see excavator.py module, 'Blended SC Pseudocode' document
#
# Created: October 06, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# TODO
#   - Saturate control action before blending law
#   - Allow operator to end control action
#   - Optimize (kill all stdout prints)
#   -
#
# Modified:
#   * October 17, 2016 - name changed to blended.py, all in place except predictor and controller
#   * October 25, 2016 - input responsive blending, mode 2
#
##########################################################################################

from excavator import *
import socket
import time
from sg_model_1101 import sg_model
from trajectories import *
from PID import PID


## I/O
# Initialize PWM/servo classes and measurement classes, note: this zeros the encoder
temp = exc_setup()
actuators = temp[0]
measurements = temp[1]

## PREDICTION
# Initialize predictor, mode 2 for input responsive, alpha = 0.5
predictor = TriggerPrediction(2, sg_model, 0.5)

## CONTROLLERS
# PI Controllers for each actuator
boom_PI = PID(1, 0.1, 0, 0, 0, 4, -4)
stick_PI = PID(1.5, 0.1, 0, 0, 0, 4, -4)
bucket_PI = PID(1, 0.1, 0, 0, 0, 4, -4)
swing_PI = PID(10, 0.1, 0, 0, 0, 4, -4)
controllers = [boom_PI, stick_PI, bucket_PI, swing_PI]

# Initialize integrator and derivator to zero
for c in controllers:
    c.setIntegrator(0)
    c.setDerivator(0)

## NETWORKING
# Networking details
HOST, PORT = '', 9999

# Create a socket (SOCK_DGRAM means a UDP socket)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

## DATA LOGGING
# Initialize DataLogger class with mode 1 for manual
filename = raw_input('Name the output file (end with .csv) or press return to disable data logging: ')
if filename == '':
    print('No data storage selected')
else:
    print('Writing headers to: ' + filename)
    data = DataLogger(3, filename)

start = time.time()

# In case packets are dropped, we intialize the joystick inputs
received_parsed = [0, 0, 0, 0]

try:
    # Connect to server and send data
    sock.bind((HOST, PORT))

    while True:
        loop_start = time.time()

        # Start by updating measurements
        for m in measurements:
            m.update_measurement()

        # # Receive joystick data from the server
        # received_joysticks = sock.recv(4096)

        # Parse data (and apply joystick deadzone)
        try:
            received_parsed = parser(sock.recv(4096), received_parsed)
        except ValueError:
            pass

        # Initial prediction step
        sg, active = predictor.update_state([received_parsed[a.js_index] for a in actuators], [m.value for m in measurements])
        # print 'Subgoal State: ', sg, active, '\n'

        # If active and need new trajectories
        if active and predictor.regen:
            # Get max duration and trajecotry coefficients
            dur = duration([m.value for m in measurements], sg_model[sg-1]['subgoal_pos'], [18, 27, 30, 0.9], [20]*4)
            coeff = quintic_coeff(dur, [m.value for m in measurements], sg_model[sg-1]['subgoal_pos'])

            # Reset integrator and differentiator
            for c in controllers:
                c.setIntegrator(0)
                c.setDerivator(0)
            
            # Set flag to not regenerate trajectories
            predictor.regen = False

            # Start a timer for the current trajectory
            active_timer = time.time()

        # If active, update PI set point and alpha
        if active:
            # Saturate time for set point
            t = time.time()-active_timer
            if t > dur:
                t = dur

            # Setpoint for controller
            for i, c in enumerate(controllers):
                c.setPoint(np.polyval(coeff[i][::-1], t))  # This version of polyval need highest degree first
            
            # Turn off swing during subgoals 1, 2, 3
            if (predictor.subgoal == 1) or (predictor.subgoal == 2) or (predictor.subgoal == 3):
                controllers[3].setPoint(measurements[3].value)

            # alpha = predictor.alpha

        # Apply blending law, alpha will either be static or zero, set duty, and update servo
        for a, c, m in zip(actuators, controllers, measurements):
            u = blending_law(received_parsed[a.js_index], c.update_sat(m.value), predictor.alpha*predictor.active, a.offset)
            # print a.actuator_name
            # print c.PID_sat
            # print received_parsed[a.js_index]
            # print u
            # u = blending_law(received_parsed[a.js_index], c.update(m.value), 0)
            a.duty_set = a.duty_span * u/(2) + a.duty_mid
            a.update_servo()
        
        try:
            data.log([loop_start-start] +                           # Run-time clock
                     received_parsed +                              # BM, ST, BK, SW joystick Cmd
                     [c.PID for c in controllers] +                 # BM, ST, BK, SW controller outputs
                     [a.duty_set for a in actuators] +              # BM, ST, BK, SW duty cycle command
                     [m.value for m in measurements] +              # BM, ST, BK, SW measurements
                     [predictor.subgoal, predictor.active])                        # Motion primitive and confidence
        except NameError:
            pass

except KeyboardInterrupt:
    print '\nQuitting'
finally:
    homing(actuators, measurements, controllers, [10, 10, 2, 0], [0.3, 0.3, 0.3, 0.05], 10)
    print '\nClosing PWM signals...'
    sock.close()
    for a in actuators:
        a.duty_set = a.duty_mid
        a.update_servo()
    time.sleep(1)
    for a in actuators:
        a.close_servo()
    if 'data' in locals():
        notes = raw_input('Notes about this trial: ')
        n = open('data/metadata.csv', 'a')
        n.write('\n' + filename + ',' + notes)
        n.close()
