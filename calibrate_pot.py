#! /usr/bin/env python

##########################################################################################
# calibrate_pot.py
#
# Creates a text file with calibration values for the string potentiometers
#
# NOTE: Samples analog value on P9_33 at certain dictated distances and creates an X-Y LUT
#
# Created: August 24, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

import Adafruit_BBIO.ADC as ADC
import time


if __name__ == "__main__":

    ADC.setup()

    # Create list of distances at which to take measurements
    dist = [i/4.0 for i in range(0, 21)]
    measure = []

    for i in dist:
        print('Move to: %f' % i)

        # Countdown
        for sec in range(4, 0, -1):
            print sec
            time.sleep(1)

        measure.append(ADC.read_raw('P9_33'))

    print measure
