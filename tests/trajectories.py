#! /usr/bin/env python

##########################################################################################
# trajectories.py
#
# This module computes the forward and inverse kinematics of the excavator,
# i.e., from actuator space to joint space and end-effector pos, and back
# Also, the sinusoidal-quintic trajectories between two points in end-effector space can be found
#
# NOTE:
#
# Created: October 18, 2016
#   - Mitchell Allain
#   - allain.mitch@gmail.com
#
# Modified:
#   *
#
##########################################################################################

import numpy as np
import math
import mat4py
import scipy.linalg as linalg


temp = mat4py.loadmat('exc.mat')
exc = temp['exc']


def forward_kin(exc, angle, cyl):
    '''This function computes the end-effector position of the excavator.

    Note: ported to Python from MATLAB "fwd_kin.m", assumed options = [0, 0]

    Args:
        exc (dict): a dict of the excavator physical parameters
        angle (float): the swing angle
        cyl (list: floats): a list of the displacements of the rods (BM, SK, BK)

    Returns:
        eef (list: float): the position of the end-effector (EEF) in (x, y, z - base frame) and the angle of the bucket (axis x4 w.r.t. x1(0?) ground axis)
    '''
    # Assign the base swing angle
    t1 = angle

    # Define lengths
    a1 = exc['a1']
    a2 = exc['a2']
    a3 = exc['a3']
    a4 = exc['a4']

    # Compute or Get joint angles
    # Boom angle
    r_c1 = cyl[0] + exc['r_cyl1']
    a_a1b = np.arccos((exc['r_o1b']**2 + exc['r_o1a']**2 - r_c1**2)/(2 * exc['r_o1b']*exc['r_o1a']))
    t2 = a_a1b - exc['a_b12'] - exc['a_a1x1']

    # Stick angle
    r_c2 = cyl[1] + exc['r_cyl2']
    a_c2d = np.arccos((exc['r_o2c']**2 + exc['r_o2d']**2 - r_c2**2)/(2 * exc['r_o2c'] * exc['r_o2d']))
    t3 = 3 * np.pi - exc['a_12c'] - a_c2d - exc['a_d23']

    # Bucket angle
    r_c3 = cyl[2] + exc['r_cyl3']
    a_efh = np.arccos((exc['r_ef']**2 + exc['r_fh']**2 - r_c3**2)/(2 * exc['r_ef'] * exc['r_fh']))
    a_hf3 = np.pi - exc['a_dfe'] - a_efh
    r_o3h = math.sqrt(exc['r_o3f']**2 + exc['r_fh']**2 - 2 * exc['r_o3f'] * exc['r_fh'] * np.cos(a_hf3))
    a_f3h = np.arccos((r_o3h**2 + exc['r_o3f']**2 - exc['r_fh']**2)/(2 * r_o3h * exc['r_o3f']))
    a_h3g = np.arccos((r_o3h**2 + exc['r_o3g']**2 - exc['r_gh']**2)/(2 * r_o3h * exc['r_o3g']))
    t4 = 3 * np.pi - a_f3h - a_h3g - exc['a_g34'] - exc['a_23d']

    c1 = np.cos(t1)
    c2 = np.cos(t2)
    c234 = np.cos(t2 + t3 + t4)
    c23 = np.cos(t2 + t3)
    s1 = np.sin(t1)
    s2 = np.sin(t2)
    s234 = np.sin(t2 + t3 + t4)
    s23 = np.sin(t2 + t3)

    # Transformation matrices from every frame to base frame
    T01 = np.array([[np.cos(t1), 0, np.sin(t1), a1 * np.cos(t1)],
                    [np.sin(t1), 0, -np.cos(t1), a1 * np.sin(t1)],
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]])

    T02 = np.array([[c1*c2, -c1*s2, s1, c1*(a2*c2+a1)],
                    [s1*c2, -s1*s2, -c1, s1*(a2*c2+a1)],
                    [s2, c2, 0, a2*s2],
                    [0, 0, 0, 1]])

    T03 = np.array([[c1*c23, -c1*s23, s1, c1*(a3*c23+a2*c2+a1)],
                    [s1*c23, -s1*s23, -c1, s1*(a3*c23+a2*c2+a1)],
                    [s23, c23, 0, a3*s23+a2*s2],
                    [0, 0, 0, 1]])

    P04 = np.array([[np.cos(t1)*(a4*c234+a3*c23+a2*np.cos(t2)+a1)],
                    [np.sin(t1)*(a4*c234+a3*c23+a2*np.cos(t2)+a1)],
                    [(a4*s234+a3*s23+a2*np.sin(t2)+a1)],
                    [1]])

    # print(P04)

    # Bucket angle; angle between x4 and x0-y0 plane
    tb = t2 + t3 + t4 - 3 * np.pi
    # T04 = np.array([[np.cos(tb) * c234, -np.cos(tb)*s234, np.sin(tb), P04[0]],
    #                 [np.sin(tb) * c234, -np.sin(tb)*s234, np.cos(tb), P04[1]],
    #                 [s234, c234, 0, P04[2]],
    #                 [0, 0, 0, P04[3]]])

    # Position and orientation of the end effector
    eef = [axis.pop() for axis in P04[0:3].tolist()]
    assert eef
    eef.append(tb)

    return eef

def inverse_kin(exc, pos):
    '''Computes the inverse kinematics from end-effector position to actuator space.

    Note: see inv_kin.m file, based on Austin's work (I believe)

    Args:
        exc (dict): a dict with all of the physical parameters of the excavator, can be imported from .mat file
        pos (list: floats): list of (x, y, z, theta) coordinates of EEF in base frame, where theta is angle of bucket from x4 to x0

    Returns:
        actuator (list: floats): list of actuator displacements, and swing angle (BM, SK, BK, SW)
    '''
    l = [0, 0, 0, 0]

    a1 = exc['a1']
    a2 = exc['a2']
    a3 = exc['a3']
    a4 = exc['a4']

    ang_B12 = exc['a_b12']
    ang_A1x1 = exc['a_a1x1']
    ang_12C = exc['a_12c']
    ang_D23 = exc['a_d23']
    ang_DFE = exc['a_dfe']
    ang_G34 = exc['a_g34']
    ang_23D = exc['a_23d']

    O1B = exc['r_o1b']
    O1A = exc['r_o1a']
    O2C = exc['r_o2c']
    O2D = exc['r_o2d']
    O3F = exc['r_o3f']
    O3G = exc['r_o3g']
    EF = exc['r_ef']
    FH = exc['r_fh']
    GH = exc['r_gh']

    # Bucket tip angle t5 = phi
    phi = pos[3] + 3 * np.pi
    cp = np.cos(phi)
    sp = np.sin(phi)

    # Base swing
    t1 = np.arctan(pos[1]/pos[0])
    c1 = np.cos(t1)
    s1 = np.sin(t1)
    l[3] = t1

    # Boom angle and Boom cylinder length
    v_o13 = [(pos[0]/c1) - a1 - a4*cp, pos[2] - a1 - a4*sp, 0]
    mag_O13 = math.sqrt(v_o13[0]**2+v_o13[1]**2)
    ang_31x1 = np.arctan(v_o13[1]/v_o13[0])
    ang_213 = np.arccos((a2**2 + mag_O13**2 - a3**2)/(2*mag_O13*a2))
    t2 = ang_31x1 + ang_213
    l[0] = math.sqrt(O1A**2 + O1B**2 - 2*np.cos(t2 + ang_B12 + ang_A1x1)*O1A*O1B) - exc['r_cyl1']

    # Stick angle and Stick cylinder length
    ang_312 = np.arccos((a3**2 + a2**2 - mag_O13**2)/(2*a3*a2))
    t3 = ang_312 + np.pi
    l[1] = math.sqrt(O2C**2 + O2D**2 - 2*O2C*O2D*np.cos(3*np.pi - t3 - ang_12C - ang_D23)) - exc['r_cyl2']

    # Bucket angle and Bucket cylinder length
    t4 = phi - t2 - t3
    ang_x33G = 2*np.pi - ang_G34 - t4
    ang_G3F = np.pi - ang_x33G - ang_23D
    FG = math.sqrt(O3F**2 + O3G**2 - 2*O3F*O3G*np.cos(ang_G3F))
    ang_3FG = np.arccos((O3F**2+FG**2 - O3G**2)/(2*O3F*FG))
    ang_HFG = np.arccos((FH**2+FG**2 - GH**2)/(2*FH*FG))

    if (t4 + ang_G34) > (2*np.pi):
        ang_HFE = ang_HFG + ang_3FG
    else:
        ang_HFE = ang_HFG - ang_3FG

    ang_EFH = np.pi - ang_DFE - ang_HFE
    l[2] = math.sqrt(EF**2 + FH**2 - 2*EF*FH*np.cos(ang_EFH)) - exc['r_cyl3']

    ang = [t1, t2, t3, t4]

    # Velocity Jacobians
    c1 = np.cos(t1)
    c2 = np.cos(t2)
    c3 = np.cos(t3)
    c4 = np.cos(t4)
    c234 = np.cos(t2 + t3 + t4)
    c23 = np.cos(t2 + t3)
    s1 = np.sin(t1)
    s2 = np.sin(t2)
    s3 = np.sin(t3)
    s4 = np.sin(t4)
    s234 = np.sin(t2 + t3 + t4)
    s23 = np.sin(t2 + t3)

    # Jacobian matrix
    # Only if non-zero end point velocity and acceleration are used.
    J = np.array([[-s1*(a4*c234+a3*c23+a2*c2+c1+a1), -c1*(a4*s234+a3*s23+a2*s2), -c1*(a4*s234+a3*s23), -c1*a4*s234],
        [c1*(a4*c234+a3*c23+a2*c2+c1+a1) -s1*(a4*s234+a3*s23+a2*s2) -s1*(a4*s234+a3*s23) -s1*a4*s234],
        [0, a4*c234+a3*c23+a2*c2, a4*c234+a3*c23, a4*c234],
        [0, s1, s1, s1],
        [0, -c1, -c1, -c1],
        [1, 0, 0, 0]])

    actuator = l
    return actuator


def quintic_coeff(t, l0, lf, t0=0, v0=0, vf=0, a0=0, af=0):
    '''Quintic trajectories from point to point in actuator space

    Args:
        t (float): duration of movement
        l0 (list: float): list of [swing starting angle, cylinder length starting points]
        lf (list: float): list of [swing endpoint, cylinder length endpoints]

    Returns:
        coeff (numpy.array): numpy array of coefficients of three cylinders and actuator
                             coefficients start with lowest order
                             coeff = array([[SW...],
                                            [BM...],
                                            [SK...],
                                            [BK...]]
    '''
    Q = np.array([[1, t0, t0**2, t0**3, t0**4, t0**5],
                  [0,  1,  2*t0,  3*t0**2,  4*t0**3, 5*t0**4],
                  [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
                  [1, t, t**2, t**3, t**4, t**5],
                  [0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4],
                  [0, 0, 2, 6*t, 12*t**2, 20*t**3]])

    L = np.array([l0,
                 [0]*4,
                 [0]*4,
                 lf,
                 [0]*4,
                 [0]*4]).transpose()

    coeff = linalg.lstsq(Q, L)[0].transpose()
    return coeff
