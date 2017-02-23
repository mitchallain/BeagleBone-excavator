#! /usr/bin/env python

##########################################################################################
# logging.py
#
# Implements simple data logger
#
# NOTE: created from excavator.py, with better API
#
# Created: February 17, 2017
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#   *
#
##########################################################################################


class DataLogger():
    '''Data logging to csv

    Args:
        mode (int): 1 for manual, 2 for autonomous, 3 for Blended
        file (obj): initialize with file object in write mode

    Attributes:
        mode (int): see above
        file (obj): file object for writing data
    '''
    def __init__(self, mode, file):
        self.mode = mode
        self.file = file
        if self.mode == 1:     # Manual mode
            self.file.write('Time,Boom Cmd,Stick Cmd,Bucket Cmd,Swing Cmd,Boom Ms,Stick Ms,Bucket Ms,Swing Ms\n')

        elif self.mode == 2:   # Autonomous mode
            self.file.write('Time,Boom Ms,Stick Ms,Bucket Ms,Swing Ms,Boom Cmd,Stick Cmd,Bucket Cmd,Swing Cmd,Boom Error,Stick Error,Bucket Error,Swing Error\n')

        elif self.mode == 3:   # Blended mode (Commands, Controllers, Blended, Measurements, Class, Probability)
            self.file.write('Time,Boom Cmd,Stick Cmd,Bucket Cmd,Swing Cmd,Boom Ctrl,Stick Ctrl,Bucket Ctrl,Swing Ctrl,Boom Blended,Stick Blended,Bucket Blended,Swing Blended,Boom Ms,Stick Ms,Bucket Ms,Swing Ms,Class,Confidence,\n')

    def log(self, data_listed):
        self.file.write(','.join(map(str, data_listed))+'\n')

    def close(self):
        self.file.close()
