#! /usr/bin/env python

##########################################################################################
# ziegler_nichols.py
#
# Using zieglers-nichols criteria to automate PID gain selection
#
# NOTE:
#
# Created: October 25, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################


from excavator import *
from PID import PID


boom_PI = PID(15, 8, 0, 0, 0, 10, -10)
stick_PI = PID(24, 10, 0, 0, 0, 10, -10)
bucket_PI = PID(12, 7.56, 0, 0, 0, 10, -10)
swing_PI = PID(10, 0.01, 0, 0, 0, 10, -10)
controllers = [boom_PI, stick_PI, bucket_PI, swing_PI]

actuators, measurements = exc_setup()


while True:
    try:
        # Read measurement and print position
        for m in measurements:
            m.update_measurement()
        print([(measurements[i].measure_type, measurements[i].value) for i in range(len(measurements))])

        # Wait for human to input gain parameters and controller index
        gain = input('Enter a list with each gain and the actuator of interest\n(i.e., [kp, ki, kd, act #]): ')
        if type(gain) != list:
            raise TypeError

        index = gain[3]
        controllers[index].Kp = gain[0]
        controllers[index].Ki = gain[1]
        controllers[index].Kd = gain[2]

        # Wait for human to input home position
        home = input('Enter the home position for the actuator: ')
        if (type(home) != float) and (type(home) != int):
            raise TypeError

        # Seek home position until user triggers stop
        homing([actuators[index]], [measurements[index]], [controllers[index]], [home], [0.2], 10)

        # Print gain and endpoint
        print(gain, [(measurements[i].measure_type, measurements[i].value) for i in range(4)])
    except KeyboardInterrupt:
        print '\nClosing PWM signals...'
        homing([actuators[3]], [measurements[3]], [controllers[3]], [0], [0.01], 10)
        for a in actuators:
            a.duty_set = a.duty_mid
            a.update_servo()
        time.sleep(1)
        for a in actuators:
            a.close_servo()
