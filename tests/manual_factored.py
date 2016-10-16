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
#
##########################################################################################

import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
import numpy as np
import socket
import time
import datetime
import os



# Networking details
HOST, PORT = '192.168.7.1', 9999

# Initialize PMM Functions (Servos and PWM BESC)
boom = Servo("P9_22", 4.939, 10.01)
arm = Servo("P8_13", 4.929, 8.861)
bucket = Servo("P8_34", 5.198, 10.03)
swing = Servo("P9_28", 4.939, 10)
actuators = (boom, arm, bucket, swing)

# Initialize measurement classes
boom_ms = Measurement('P9_37', 'boom')
stick_ms = 

if __name__ == "__main__":

    # Measurement initialization
    ADC.setup()

    # Create a socket (SOCK_STREAM means a TCP socket)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

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

        f.write('Time,Boom JS,Stick JS,Bucket JS,Swing JS,Boom Cmd,Stick Cmd,Bucket Cmd,Swing Cmd,Boom Ms,Stick Ms,Bucket Ms,Swing Ms\n')

    start = time.time()

    try:
        # Connect to server and send data
        sock.connect((HOST, PORT))
        while True:

            # Receive data from the server
            received_joysticks = sock.recv(4096)

            # Parse data (and apply joystick deadzone)
            try:
                received_parsed = parser(received_joysticks)
            except ValueError:
                pass

            # New duty cycles
            boom_duty_cycle_input = boom.duty_span*(received_parsed[2] + 1)/(2) + boom.duty_min
            arm_duty_cycle_input = arm.duty_span*(received_parsed[3] + 1)/(2) + arm.duty_min
            bucket_duty_cycle_input = bucket.duty_span*(-received_parsed[0] + 1)/(2) + bucket.duty_min
            swing_duty_cycle_input = swing.duty_span*(received_parsed[1] + 1)/(2) + swing.duty_min

            # Data Logging
            excavator.data_log([x.getValue() for x in measurement])
            
            try:
                f.write(str(time.time()-start) + ',' +          # Time
                        str(received_parsed[2]) + ',' +         # Boom JS
                        str(received_parsed[3]) + ',' +         # Stick JS
                        str(received_parsed[0]) + ',' +         # Bucket JS
                        str(received_parsed[1]) + ',' +         # Swing JS
                        str(boom_duty_cycle_input) + ',' +      # Boom Cmd
                        str(arm_duty_cycle_input) + ',' +       # Stick Cmd
                        str(bucket_duty_cycle_input) + ',' +    # Bucket Cmd
                        str(swing_duty_cycle_input) + ',' +     # Swing Cmd
                        str(interpolate(ADC.read_raw('P9_37'), 'boom')) + ',' +     # Boom Ms
                        str(interpolate(ADC.read_raw('P9_33'), 'stick')) + ',' +    # Stick Ms
                        str(interpolate(ADC.read_raw('P9_35'), 'bucket')) + '\n')   # Bucket Ms
                        # Missing Swing Ms, see manual_ol_swing.py
            except NameError:
                pass

            # Update PWM
            print boom_duty_cycle_input, arm_duty_cycle_input, bucket_duty_cycle_input, swing_duty_cycle_input
            # print boom_duty_cycle_input, swing_duty_cycle_input
            PWM.set_duty_cycle(boom.servo_pin, boom_duty_cycle_input)
            PWM.set_duty_cycle(arm.servo_pin, arm_duty_cycle_input)
            PWM.set_duty_cycle(bucket.servo_pin, bucket_duty_cycle_input)
            PWM.set_duty_cycle(swing.servo_pin, swing_duty_cycle_input)

    except KeyboardInterrupt:
        print '\nQuitting'
    finally:
        print '\nClosing PWM signals...'
        sock.close()
        boom.close_servo()
        arm.close_servo()
        bucket.close_servo()
        swing.close_servo()
