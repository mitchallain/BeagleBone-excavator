#! /usr/bin/env python

##########################################################################################
# manual_no_measure.py
#
# Manual operation without accessing the analog in
#
# NOTE:
#
# Created: March 07, 2017
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   * 
#
##########################################################################################

from excavator import *
import socket
import time


# Networking details
HOST, PORT = '', 9999

if __name__ == "__main__":

    # Initialize PWM/servo classes and measurement classes, note: this zeros the encoder
    actuators = actuator_setup()

    # Create a socket (SOCK_DGRAM means a UDP socket)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    start = time.time()
    received_parsed = [0, 0, 0, 0]

    try:
        # Connect to server and send data
        sock.bind((HOST, PORT))

        while True:
            loop_start = time.time()

            # Receive data from the server
            received_joysticks = sock.recv(4096)

            # Parse data (and apply joystick deadzone)
            try:
                received_parsed = parse_joystick(received_joysticks, received_parsed)
            except ValueError:
                pass
            
            # print received_parsed

            for a in actuators:
                a.duty_set = a.duty_span*(received_parsed[a.js_index]+1)/(2) + a.duty_min
                a.update_servo()

            print([a.duty_set for a in actuators])

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
