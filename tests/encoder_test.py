from bbio.libraries.RotaryEncoder import RotaryEncoder
import time
import excavator
import socket

temp = excavator.exc_setup()
a = temp[0]

encoder = RotaryEncoder(RotaryEncoder.EQEP2)
encoder.enable()

raw_input('Press enter to continue...')
encoder.setAbsolute()
encoder.zero()

# Networking details
HOST, PORT = '192.168.7.1', 9999

# Create a socket (SOCK_STREAM means a TCP socket)
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

received_parsed = [0, 0, 0, 0]

try:
    while True:
        received_joysticks = sock.recv(4096)
        try:
            received_parsed = parser(received_joysticks, received_parsed)
        except ValueError:
            pass
        
        a[3].duty_set = a[3].duty_span*(-received_parsed[1] + 1)/(2) + a[3].duty_min
        a[3].update_servo()
        
        print "encoder position : "+str(encoder.getPosition())
        # time.sleep(1)
except KeyboardInterrupt:
    print('Quitting...')
finally:
    sock.close()
    for a in actuators:
        a.close_servo()