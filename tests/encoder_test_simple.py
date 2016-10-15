from bbio.libraries.RotaryEncoder import RotaryEncoder
import time

encoder = RotaryEncoder(RotaryEncoder.EQEP1)
encoder.enable()

raw_input('Press enter to continue...')
encoder.setAbsolute()
encoder.zero()

while True:
    print "encoder position : "+str(encoder.getPosition())
    time.sleep(1)
