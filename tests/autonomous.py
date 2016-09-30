#! /usr/bin/env python

##########################################################################################
# autonomous.py
#
# Derived from manual.py
# Autonomous operation of the excavator. Records data and timestamps.
#
# NOTE: SERVO CLASS, INTERPOLATE FUNCTION, AND MISC DEPENDENCIES IMPORTED FROM MANUAL.PY
#
# Created: September 27, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   * September 28, 2016 - changed to autonomous.py, implemented poly func
#
##########################################################################################

from manual import *
import numpy.polynomial.polynomial as poly


def saturate(low, high, value):
    return max(low, min(high, value))

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

    f.write('Time,Boom Error,Stick Error,Bucket Error,Swing Error,Boom Cmd,Stick Cmd,Bucket Cmd,Swing Cmd,Boom Ms,Stick Ms,Bucket Ms,Swing Ms\n')

# Initialize trajectory (coeff beginning with lowest order)
coeff = (74.7, 8, 6, -2)

# Initialize the PWM signals
boom_duty_cycle_input = boom.duty_mid
arm_duty_cycle_input = arm.duty_mid
bucket_duty_cycle_input = bucket.duty_mid
swing_duty_cycle_input = swing.duty_mid

PWM.set_duty_cycle(boom.servo_pin, boom_duty_cycle_input)
PWM.set_duty_cycle(arm.servo_pin, arm_duty_cycle_input)
PWM.set_duty_cycle(bucket.servo_pin, bucket_duty_cycle_input)
PWM.set_duty_cycle(swing.servo_pin, swing_duty_cycle_input)

start = time.time()
loopend = 0

try:
    # # Connect to server and send data
    # sock.connect((HOST, PORT))
    while loopend < 4:
        loopstart = time.time()

        # Measurement
        boom_ms = interpolate(ADC.read_raw('P9_37'), 'boom')
        stick_ms = interpolate(ADC.read_raw('P9_33'), 'stick')
        bucket_ms = interpolate(ADC.read_raw('P9_35'), 'bucket')

        # Error signal
        ref = poly.polyval((time.time()-start), coeff)
        e = ref - boom_ms

        # Generate control signals
        boom_duty_cycle = boom.duty_span*((e/10) + 1)/(2) + boom.duty_min
        boom_duty_cycle_input = saturate(boom_duty_cycle, 4.939, 10.01)
        PWM.set_duty_cycle(boom.servo_pin, boom_duty_cycle_input)
        print(e, boom_duty_cycle_input)

        # Data Logging
        try:
            f.write(str(loopstart - start) + ',' +          # Time
                    str(e) + ',' +                          # Boom e
                    str(0) + ',' +                          # Stick e
                    str(0) + ',' +                          # Bucket e
                    str(0) + ',' +                          # Swing e
                    str(boom_duty_cycle_input) + ',' +      # Boom Cmd
                    str(arm_duty_cycle_input) + ',' +       # Stick Cmd
                    str(bucket_duty_cycle_input) + ',' +    # Bucket Cmd
                    str(swing_duty_cycle_input) + ',' +     # Swing Cmd
                    str(boom_ms) + ',' +                    # Boom Ms
                    str(stick_ms) + ',' +                   # Stick Ms
                    str(bucket_ms) + '\n')                  # Bucket Ms
        except NameError:
            pass
        
        loopend = time.time()-start
except KeyboardInterrupt:
    print '\nQuitting'
finally:
    print '\nClosing PWM signals...'
    PWM.set_duty_cycle(boom.servo_pin, boom.duty_mid)
    time.sleep(1)
    boom.close_servo()
    arm.close_servo()
    bucket.close_servo()
    swing.close_servo()