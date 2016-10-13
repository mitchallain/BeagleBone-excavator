#! /usr/bin/env python

##########################################################################################
# calibrate.py
#
# Calibrates potentiometer on pin 33 (SK), 35 (BK), and 37 (BM)
#
# NOTE: Paste output into excavator.py update_measurement function
#
# Created: September 12, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   * October 12, 2016 - changed name and recalibrated
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
            potMeasure.append(ADC.read_raw('P9_33'))
            calMeasure.append(user)

    print calMeasure
    print potMeasure
