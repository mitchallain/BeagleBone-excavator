#! /usr/bin/env python

##########################################################################################
# calibrate_bucket.py
#
# Calibrates bucket potentiometer on pin 35
#
# NOTE: Extra notes
#
# Created: September 12, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

import Adafruit_BBIO.ADC as ADC


if __name__ == "__main__":

    ADC.setup()

    calMeasure = []
    potMeasure = []

    while True:
        user = input("Enter a distance or 'exit': ")

        if user == 'exit':
            break
        else:
            potMeasure.append(ADC.read_raw('P9_35'))
            calMeasure.append(user)

    print calMeasure
    print potMeasure
