#! /usr/bin/env python

##########################################################################################
# potentiometer_read.py
#
# Reads the analog input on pin 33 from a potentiometer and prints to screen
#
# NOTE: Will be used to develop absolute position sensing with multiturn encoders fixed
#       to cylinders, rolling on rods on excavator.
#
# Created: August 10, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

import Adafruit_BBIO.ADC as ADC
import time
import numpy as np

if __name__ == "__main__":

    ADC.setup()

    try:
        while True:
            print(ADC.read_raw('P9_35'))    # print pin value
            time.sleep(2)     # sleep for a tenth of a second
    except KeyboardInterrupt:
        print('\nQuitting')
