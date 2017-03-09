#! /usr/bin/env python

##########################################################################################
# manual_with_triggering.py
#
# Derived from manual.py
# Manual operation of the excavator.
# Records positions in the actuator space when the right joystick trigger is pulled
#
# NOTE:
#
# Created: March 09, 2017 from manual.py
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#
##########################################################################################

import excavator as exc
import socket
import time


def close_io(socket, actuators):
    print '\nClosing PWM signals...'
    socket.close()
    for a in actuators:
        a.duty_set = a.duty_mid
        a.update_servo()
    time.sleep(1)
    for a in actuators:
        a.close_servo()


def parse_joystick(received, received_parsed):
    '''Parse joystick data from server_02.py, and convert to float'''
    deadzone = 0.2
    toggle_invert = [1, 1, -1, -1]  # Invert [BM, SK, BK, SW] joystick
    try:
        received = received.translate(None, "[( )]").split(',')
        for axis in xrange(len(received) - 1):
            if (float(received[axis]) > deadzone) or (float(received[axis]) < -deadzone):
                received_parsed[axis] = float(received[axis])*toggle_invert[axis]
            else:
                received_parsed[axis] = 0
        return received_parsed[0:4], int(received[4])
    except ValueError:
        print '\nValue Error'
        raise ValueError


# Networking details
HOST, PORT = '', 9999

if __name__ == "__main__":

    # Initialize PWM/servo classes and measurement classes, note: this zeros the encoder
    temp = exc.exc_setup()
    actuators = temp[0]
    measurements = temp[1]

    # Create a socket (SOCK_DGRAM means a UDP socket)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Initialize DataLogger class with mode 1 for manual
    filename = raw_input('Name the output file (end with .csv) or press return to disable data logging: ')
    if filename == '':
        print('No data storage selected')
    else:
        print('Writing headers to: ' + filename)
        data = exc.DataLogger(1, filename)

    start = time.time()
    received_parsed = [0, 0, 0, 0]
    trigger = 0

    try:
        # Connect to server
        sock.bind((HOST, PORT))

        while True:
            loop_start = time.time()

            # Receive data from the server
            received_joysticks = sock.recv(4096)

            # Parse data (and apply joystick deadzone)
            try:
                received_parsed, trigger = parse_joystick(received_joysticks, received_parsed)
            except ValueError:
                trigger = 0
                pass

            print(received_parsed, trigger)

            for a in actuators:
                a.duty_set = a.duty_span*(received_parsed[a.js_index]+1)/(2) + a.duty_min
                a.update_servo()

            for m in measurements:
                m.update_measurement()

            if trigger == 1:
                # Data logging mode 2 (manual)
                try:
                    data.log([loop_start-start] +                   # Run-time clock
                             [a.duty_set for a in actuators] +      # BM, ST, BK, SW Duty Cycle Cmd
                             [m.value for m in measurements])       # BM, ST, BK, SW Measurements
                except NameError:
                    pass

            # Soft timing
            while (time.time() - loop_start) < 0.05:
                pass

    except KeyboardInterrupt:
        print '\nQuitting'
    finally:
        if 'data' in locals():
            data.close()
