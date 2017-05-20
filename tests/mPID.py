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
    Discrete PID control

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
        self.e = self.setpoint - measurement

        self.proportional = self.kp * self.e
        self.integral = np.clip(self.integral + self.e * self.ki, self.i_min, self.i_max)
        self.derivative = (self.e - self.e1) * self.kd

        self.e1 = self.e

        self.pid = self.proportional + self.integral + self.derivative
        return self.pid


class TustinPID():
    """
    Discrete PID control, performs bilinear transformation on standard PID controller
        with gains provided using the sample period provided.

    Tustin's Transformation:
            2 (z - 1)
        s = ---------
            T (z + 1)

    Args:
        T (float): sample time
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
        A (float): coeff 1
        B (float): coeff 2
        C (float): coeff 3
        e (float): error
        e1 (float): error
        e2 (float): error
        u (float): input
        u1 (float): input
        u2 (float): input
        out_max (float): maximum output value
        out_min (float): minimum output value
        setpoint (float): set point for controller
    """

    def __init__(self, T, kp, ki=0.0, kd=0.0, out_max=1.0, out_min=-1.0):
        self.A = (4.0 * kd + ki * (T**2) + 2.0 * kp * T) / (2.0 * T)
        self.B = (2 * ki * (T**2) - 8.0 * kd) / (2.0 * T)
        self.C = (4.0 * kd + ki * (T**2) - 2.0 * kp * T) / (2.0 * T)

        self.out_max = out_max
        self.out_min = out_min

        self.setpoint = 0.0
        self.e = 0.0
        self.e1 = 0.0
        self.e2 = 0.0
        self.u = 0.0
        self.u1 = 0.0
        self.u2 = 0.0

    def update(self, measurement):
        ''' Calculate PID output '''
        self.e = self.setpoint - measurement
        self.u = self.A * self.e + self.B * self.e1 + self.C * self.e2 + self.u2

        # Saturate
        self.u = np.clip(self.u, self.out_min, self.out_max)

        # Housekeeping
        self.e2 = self.e1
        self.u2 = self.u1
        self.e1 = self.e
        self.u1 = self.u

        return self.u
