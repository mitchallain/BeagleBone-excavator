#! /usr/bin/env python

##########################################################################################
# manual.py
#
# Derived from manual_unfactored.py
# Manual operation of the excavator. Records data and time stamps.
#
# NOTE:
#
# Created: September 14, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   * October 06, 2016, deadzone joysticks in parser function
#   * October 11, 2016, factored out code into measurement class, modified servo class,
#   * October 16, 2016, finished factoring, renamed to manual.py
#
##########################################################################################

from excavator import *
import socket
import time


# Networking details
HOST, PORT = '192.168.7.1', 9999

if __name__ == "__main__":

    # Initialize PWM/servo classes and measurement classes, note: this zeros the encoder
    temp = exc_setup()
    actuators = temp[0]
    measurements = temp[1]

    # Create a socket (SOCK_STREAM means a TCP socket)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Initialize DataLogger class with mode 1 for manual
    filename = raw_input('Name the output file (end with .csv) or press return to disable data logging: ')
    if filename == '':
        print('No data storage selected')
    else:
        print('Writing headers to: ' + filename)
        data = DataLogger(1, filename)

    start = time.time()
    received_parsed = [0, 0, 0, 0]

    try:
        # Connect to server and send data
        sock.connect((HOST, PORT))

        while True:
            loop_start = time.time()

            # Receive data from the server
            received_joysticks = sock.recv(4096)

            # Parse data (and apply joystick deadzone)
            try:
                received_parsed = parser(received_joysticks, received_parsed)
            except ValueError:
                pass

            for a in actuators:
                a.duty_set = a.duty_span*(received_parsed[a.js_index]+1)/(2) + a.duty_min
                a.update_servo()

            print([a.duty_set for a in actuators])

            for m in measurements:
                m.update_measurement()

            # Data logging mode 2 (manual)
            try:
                data.log([loop_start-start] +                   # Run-time clock
                         [a.duty_set for a in actuators] +      # BM, ST, BK, SW Duty Cycle Cmd
                         [m.value for m in measurements])       # BM, ST, BK, SW Measurements
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
