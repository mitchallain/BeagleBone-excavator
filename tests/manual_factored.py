#! /usr/bin/env python

##########################################################################################
# manual_factored.py
#
# Derived from BBB_tm_Full.py
# Manual operation of the excavator. Begins recording data and timestamps
#
# NOTE: THIS WILL SERVE AS A DEPENDENCY FOR OTHER MAIN TEST SCRIPTS, ACT WITH
#       CAUTION
#
# Created: September 14, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   * October 06, 2016, deadzone joysticks in parser function
#   * October 11, 2016, factored out code into measurement class, modified servo class, MUCH BETTER
#
##########################################################################################

import Adafruit_BBIO.PWM as PWM
import Adafruit_BBIO.ADC as ADC
import numpy as np
import socket
import time
import datetime
import os


class Servo():
    '''Servo class stores pin info and duty limits'''
    def __init__(self, servo_pin, duty_min, duty_max):
        self.duty_min = duty_min
        self.duty_max = duty_max
        self.duty_span = self.duty_max - self.duty_min
        self.duty_mid = ((90.0 / 180) * self.duty_span + self.duty_min)
        self.duty_set = self.duty_mid

        self.servo_pin = servo_pin
        print 'starting servo PWM'
        PWM.start(self.servo_pin, self.duty_mid, 50.625)

    def update_servo(self):
        '''Saturate duty cycle at limits'''
        duty_cycle_clamp = max(self.duty_min, min(self.duty_max, self.duty_set))
        PWM.set_duty_cycle(duty_cycle_clamp)

    def close_servo(self):
        PWM.stop(self.servo_pin)
        PWM.cleanup()


class Measurement():
    '''Contains current measurement and pin info with update methods'''
    def __init__(self, GPIO_pin, measure_type):
        ADC.setup()
        self.GPIO_pin = GPIO_pin
        self.measure_type = measure_type
        self.lookup = {'boom': [[536.0, 564.0, 590.0, 627.0, 667.0, 707.0, 753.0, 806.0, 845.0, 872.0, 895.0, 916.0, 923.0],    # BM Analog Input
                                [0, 7.89, 14.32, 22.64, 33.27, 44.05, 56.85, 73.62, 87.54, 98.1, 107.14, 115.34, 118.27]],      # BM Displacement mm
                       'stick': [[558.0, 624.0, 644.0, 685.0, 724.0, 757.0, 786.0, 826.0, 857.0, 891.0, 918.0, 950.0, 977.0, 998.0, 1010.0, 1020.0],    # SK Analog Input
                                [0, 16.28, 21.27, 31.54, 42.06, 51.6, 59.79, 72.94, 83.06, 94.96, 105.03, 117.58, 128.79, 138.52, 143.84, 148]],        # SK Displacement mm
                       'bucket': [[176.0, 263.0, 308.0, 358.0, 417.0, 473.0, 525.0, 576.0, 613.0, 633.0],
                                [0, 17.27, 25.87, 36.6, 50.65, 64.64, 77.93, 91.73, 102.96, 109.24]]}

    def update_measurement(self):
        self.value = np.interp(ADC.read_raw(self.GPIO_pin), self.lookup[self.measure_type][0], self.lookup[self.measure_type][1])


# def interpolate(pot_reading, actuator):
#     '''Use lookup tables to interpolate actuator position in mm from string pot voltage'''
#     if actuator == 'boom':
#         y = [0, 7.89, 14.32, 22.64, 33.27, 44.05, 56.85, 73.62, 87.54, 98.1, 107.14, 115.34, 118.27]
#         x = [536.0, 564.0, 590.0, 627.0, 667.0, 707.0, 753.0, 806.0, 845.0, 872.0, 895.0, 916.0, 923.0]
#         return np.interp(pot_reading, x, y)
#     elif actuator == 'stick':
#         y = [0, 16.28, 21.27, 31.54, 42.06, 51.6, 59.79, 72.94, 83.06, 94.96, 105.03, 117.58, 128.79, 138.52, 143.84, 148]
#         x = [558.0, 624.0, 644.0, 685.0, 724.0, 757.0, 786.0, 826.0, 857.0, 891.0, 918.0, 950.0, 977.0, 998.0, 1010.0, 1020.0]
#         return np.interp(pot_reading, x, y)
#     elif actuator == 'bucket':
#         y = [0, 17.27, 25.87, 36.6, 50.65, 64.64, 77.93, 91.73, 102.96, 109.24]
#         x = [176.0, 263.0, 308.0, 358.0, 417.0, 473.0, 525.0, 576.0, 613.0, 633.0]
#         return np.interp(pot_reading, x, y)


def parser(received, received_parsed):
    '''Parse joystick data from server_02.py, and convert to float'''
    deadzone = 0.1
    try:
        received = received.translate(None, "[( )]").split(',')
        for axis in range(len(received)):
            if (float(received[axis]) > deadzone) or (float(received[axis]) < -deadzone):
                received_parsed[axis] = float(received[axis])
            else:
                received_parsed[axis] = 0
        return received_parsed
    except ValueError:
        print '\nValue Error'
        raise ValueError


def name_date_time(file_name):
    '''Returns string of format __file__ + '_mmdd_hhmm.csv' '''
    n = datetime.datetime.now()
    data_stamp = os.path.basename(file_name)[:-3] + '_' + n.strftime('%m%d_%H%M')+'.csv'
    return data_stamp


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
