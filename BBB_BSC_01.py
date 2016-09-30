#! /usr/bin/env python

##########################################################################################
# BBB_BSC_01.py
#
# This code will call the prediction algorithms from the OSU/UofI group and 
# execute the optimal control derivation, finally blending it with the user input
#
# NOTE:
#
# Created: September 12, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

import Adafruit_BBIO.PWM as PWM
import socket


class Servo():

    def __init__(self, servo_pin, duty_min, duty_max):
        """Dr. Vaughan's Servo class for creating PWM signals"""
        self.duty_min = duty_min
        self.duty_max = duty_max
        self.duty_span = self.duty_max - self.duty_min
        self.duty_mid = ((90.0 / 180) * self.duty_span + self.duty_min)

        self.servo_pin = servo_pin
        print 'starting servo PWM'
        PWM.start(self.servo_pin, self.duty_mid, 50.625)

    def close_servo(self):
        PWM.stop(self.servo_pin)
        PWM.cleanup()


if __name__ == "__main__":

    # Functions (Servos and PWM BESC)
    boom = Servo("P9_22", 4.939, 10.01)
    arm = Servo("P8_13", 4.929, 8.861)
    bucket = Servo("P8_34", 5.198, 10.03)
    swing = Servo("P9_28", 4.939, 10)

    # Networking details
    HOST, PORT = '192.168.7.1', 9999

    # Create a socket (SOCK_STREAM means a TCP socket)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        # Connect to server and send data
        sock.connect((HOST, PORT))
        while True:

            # Receive data from the server
            received = sock.recv(4096)

            try:
            # Parse that junk
                received = received.translate(None, "[( )]").split(',')
                received_parsed = [float(i) for i in received]
            except ValueError:
                print '\nValue Error'
                pass
            
            # New duty cycles
            boom_duty_cycle_input = boom.duty_span*(received_parsed[2] + 1)/(2) + boom.duty_min
            arm_duty_cycle_input = arm.duty_span*(received_parsed[3] + 1)/(2) + arm.duty_min
            bucket_duty_cycle_input = bucket.duty_span*(-received_parsed[0] + 1)/(2) + bucket.duty_min
            swing_duty_cycle_input = swing.duty_span*(received_parsed[1] + 1)/(2) + swing.duty_min
            
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
