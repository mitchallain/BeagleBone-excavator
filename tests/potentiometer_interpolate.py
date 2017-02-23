#! /usr/bin/env python

##########################################################################################
# potentiometer_interpolate.py
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

    # Lookup table from calibrate_pot.py (measure_dist in mm)
    sp_analog = [819.0, 844.0, 882.0, 929.0, 963.0, 993.0, 1021.0, 1051.0, 1081.0, 1116.0, 1150.0, 1176.0, 1201.0, 1223.0, 1242.0, 1252.0]
    measure_dist = [0.0, 5.7, 13.74, 22.37, 30.04, 37.4, 44.94, 54.55, 65.0, 77.5, 90.74, 102.19, 114.67, 128.21, 139.23, 147.6]

    try:
        while True:
            print(np.interp(ADC.read_raw('P9_33'), sp_analog, measure_dist))    # print pin value
            time.sleep(0.1)     # sleep for a tenth of a second
    except KeyboardInterrupt:
        print('\nQuitting')
