#! /usr/bin/env python

##########################################################################################
# steps.py
#
# Derived from manual_ol_swing.py
# Applies step input at 100% duty to each function and measures response
#
# NOTE: Ctrl+C if malfunction
#       set duration; will go both directions
#       angle = angle - d_angle
#       * positive angle is counter-clockwise
#
# Created: September 19, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

from manual import *


def swing_update(swing_js, time, old_swing_angle):
    if swing_js > 0.01 or swing_js < -0.01:
            new_swing_angle = old_swing_angle - (time)*swing_js*0.5
    return new_swing_angle


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

    # Initialize swing_angle and swing_start
    swing_angle = 0.0
    swing_start = time.time()
    received_parsed = [0, 0, 0, 0]

    # start the data clock
    start = time.time()

    # Duration
    duration = 1.5  # seconds

    try:
        boom_duty_cycle_input = boom.duty_mid
        arm_duty_cycle_input = arm.duty_mid
        bucket_duty_cycle_input = bucket.duty_mid
        swing_duty_cycle_input = swing.duty_mid

        PWM.set_duty_cycle(boom.servo_pin, boom_duty_cycle_input)
        PWM.set_duty_cycle(arm.servo_pin, arm_duty_cycle_input)
        PWM.set_duty_cycle(bucket.servo_pin, bucket_duty_cycle_input)
        PWM.set_duty_cycle(swing.servo_pin, swing_duty_cycle_input)

        # boom duty max
        boom_duty_cycle_input = boom.duty_span*1 + boom.duty_min
        PWM.set_duty_cycle(boom.servo_pin, boom_duty_cycle_input)

        while ((time.time()-start) < duration):
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
                        str(interpolate(ADC.read_raw('P9_35'), 'bucket')) + ',' +   # Bucket Ms
                        str(swing_angle) + '\n')                # Swing Ms
            except NameError:
                print('NameError')
                pass
            time.sleep(0.01)

        # boom duty mid
        boom_duty_cycle_input = boom.duty_span*0.5 + boom.duty_min
        PWM.set_duty_cycle(boom.servo_pin, boom_duty_cycle_input)
        time.sleep(1)

        # boom duty min
        boom_duty_cycle_input = boom.duty_min
        PWM.set_duty_cycle(boom.servo_pin, boom_duty_cycle_input)

        # Mid time marker
        mid = time.time()

        while ((time.time()-mid) < duration):
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
                        str(interpolate(ADC.read_raw('P9_35'), 'bucket')) + ',' +   # Bucket Ms
                        str(swing_angle) + '\n')                # Swing Ms
            except NameError:
                print('NameError')
                pass
            time.sleep(0.01)

        # boom duty mid
        boom_duty_cycle_input = boom.duty_span*0.5 + boom.duty_min

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
