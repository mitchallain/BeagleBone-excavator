#! /usr/bin/env python

##########################################################################################
# manual_ol_swing.py
#
# Derived from BBB_tm_Full.py
# Manual operation of the excavator. Begins recording data and timestamps
#
# NOTE: OL swing estimation achieved with:
#       d_angle = time*(current_JS{[-1, 1]}*(angular_vel_full{rad/s})
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

def swing_update(swing_js, time, swing_angle):
    if swing_js > 0.01 or swing_js < -0.01:
            swing_angle = swing_angle - (time)*swing_js*0.5
    return swing_angle


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
    
    try:
        # Connect to server and send data
        sock.connect((HOST, PORT))
        while True:
            # Old swing joystick input for estimator
            old_swing_js = received_parsed[1]
            
            # Receive data from the server
            received_joysticks = sock.recv(4096)
    
            # Parse data
            try:
                received_parsed = parser(received_joysticks)
            except ValueError:
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
            
            # Swing estimator needs clock right before and after update.
            swing_end = time.time()
            swing_angle = swing_update(old_swing_js, (swing_end-swing_start), swing_angle)
    
            # Set swing PWM and update clock
            PWM.set_duty_cycle(swing.servo_pin, swing_duty_cycle_input)
            swing_start = time.time()
    
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
                        str(interpolate(ADC.read_raw('P9_35'), 'bucket')) + ',' +   # Bucket Ms
                        str(swing_angle) + '\n')                # Swing Ms
            except NameError:
                pass
    
    except KeyboardInterrupt:
        print '\nQuitting'
    finally:
        print '\nClosing PWM signals...'
        sock.close()
        boom.close_servo()
        arm.close_servo()
        bucket.close_servo()
