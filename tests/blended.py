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
# Modified:
#   * October 17, 2016 - name changed to blended.py, all in place except predictor and controller
#
##########################################################################################

from excavator import *
import socket
import time
from sg_model import sg_model


# Networking details
HOST, PORT = '', 9999

# Initialize PWM/servo classes and measurement classes, note: this zeros the encoder
temp = exc_setup()
actuators = temp[0]
measurements = temp[1]

# Initialize predictor, mode 0, alpha = 0.5
predictor = TriggerPrediction(0, sg_model, 0.5)

# Create a socket (SOCK_DGRAM means a UDP socket)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Initialize DataLogger class with mode 1 for manual
filename = raw_input('Name the output file (end with .csv) or press return to disable data logging: ')
if filename == '':
    print('No data storage selected')
else:
    print('Writing headers to: ' + filename)
    data = DataLogger(3, filename)

start = time.time()
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

        # Prediction step
        print predictor.update_state([received_parsed[a.js_index] for a in actuators], [m.value for m in measurements]), ["%.2f" % m.value for m in measurements]

        for a in actuators:
            a.duty_set = a.duty_span*(received_parsed[a.js_index]+1)/(2) + a.duty_min

        # # Calculate controller assistance for nominal tasks, must be in motion and above estimation confidence threshold
        # if (predictor.primitive != 0) and (predictor.confidence > predictor.blend_threshold):
        #     for i in xrange(3):
        #         controllers[i].setPoint(predictor.endpoints[i])
        #         actuators[i].duty_set = blending_law(controllers[i], received_parsed[a.js_index], predictor.alpha)

        for a in actuators:
            a.update_servo()

        # Data logging mode 2 (manual)
        try:
            data.log([loop_start-start] +                           # Run-time clock
                     received_parsed +                              # BM, ST, BK, SW joystick Cmd
                     [c.PID for c in controllers] +                 # BM, ST, BK, SW controller outputs
                     [a.duty_set for a in actuators] +              # BM, ST, BK, SW duty cycle command
                     [m.value for m in measurements] +              # BM, ST, BK, SW measurements
                     [predictor.primitive, predictor.confidence])   # Motion primitive and confidence
        except NameError:
            pass

except KeyboardInterrupt:
    print '\nQuitting'
finally:
    print '\nClosing PWM signals...'
    sock.close()
    for a in actuators:
        a.duty_set = a.duty_mid
        a.update_servo()
    time.sleep(1)
    for a in actuators:
        a.close_servo()
