#! /usr/bin/env python

##########################################################################################
# mPID.py
#
# PID class definition
#
# NOTE: replaces PID.py with bad naming and structure
#
# Created: May 18, 2017
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################


import numpy as np


class PID:
    """
    Discrete PID control, adjusts ki and kd gains for sample period

    Args:
        kp (float): proportional gain
        ki (float): integral gain
        kd (float): derivative gain
        i_max (float): maximum integrator value
        i_min (float): minimum integrator value
        out_max (float): maximum output value
        out_min (float): minimum output value

    Attributes:
        kp (float):proportional gain
        ki (float): integral gain
        kd (float): derivative gain
        proportional (float): proportional term
        derivative (float): derivative term
        integral (float): integral term
        i_max (float): maximum integrator value
        i_min (float): minimum integrator value
        out_max (float): maximum output value
        out_min (float): minimum output value
        setpoint (float): set point for controller
        error (float): error, set_point - current_value
        pid (float): output of controller
    """

    def __init__(self, ts, kp, ki=0.0, kd=0.0, i_max=10, i_min=-10, out_max=1.0, out_min=-1.0):
        self.kp = kp
        self.ki = ki * ts
        self.kd = kd / ts

        self.i_max = i_max
        self.i_min = i_min
        self.out_max = out_max
        self.out_min = out_min

        self.setpoint = 0.0
        self.e = 0.0
        self.e1 = 0.0

        self.proportional = 0
        self.integral = 0
        self.derivative = 0

    def update(self, measurement):
        ''' Calculate PID output '''
        self.e = self.set_point - measurement
        self.error_int += self.e

        self.proportional = self.kp * self.error
        self.integral = np.clip(self.error_int * self.ki, self.i_min, self.i_max)
        self.derivative = (self.e - self.e1) * self.kd

        self.e1 = self.e

        self.pid = self.proportional + self.integral + self.derivative
        return self.pid
