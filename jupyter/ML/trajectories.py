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
#   * February 08, 2017 - duplicated trajectories to create analysis version NEED TO RESOLVE DUPES
#   * Added exc_draw func to illustrate exc pose on 3D axis
#
##########################################################################################

import numpy as np
import math
import mat4py
import numpy.linalg as linalg

## Need to install mat4py on BBB
temp = mat4py.loadmat('exc.mat')
exc = temp['exc']


def forward_kin(exc, angle, cyl):
    '''This function computes the end-effector position of the excavator.
    BASE FRAME IS ATTACHED TO SWING MOTOR AT LEVEL OF BOOM JOINT

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
    # T01 = np.array([[np.cos(t1), 0, np.sin(t1), a1 * np.cos(t1)],
    #                 [np.sin(t1), 0, -np.cos(t1), a1 * np.sin(t1)],
    #                 [0, 1, 0, 0],
    #                 [0, 0, 0, 1]])

    # T02 = np.array([[c1*c2, -c1*s2, s1, c1*(a2*c2+a1)],
    #                 [s1*c2, -s1*s2, -c1, s1*(a2*c2+a1)],
    #                 [s2, c2, 0, a2*s2],
    #                 [0, 0, 0, 1]])

    # T03 = np.array([[c1*c23, -c1*s23, s1, c1*(a3*c23+a2*c2+a1)],
    #                 [s1*c23, -s1*s23, -c1, s1*(a3*c23+a2*c2+a1)],
    #                 [s23, c23, 0, a3*s23+a2*s2],
    #                 [0, 0, 0, 1]])

    P04 = np.array([[np.cos(t1)*(a4*c234+a3*c23+a2*np.cos(t2)+a1)],
                    [np.sin(t1)*(a4*c234+a3*c23+a2*np.cos(t2)+a1)],
                    [(a4*s234+a3*s23+a2*np.sin(t2))],
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


def duration(q0, qf, vmax, amax):
    '''Function to find duration of quintic trajectories for each actuator

    Args:
        q0 (list: float): start position
        qf (list: float): endpoint position
        vmax (list: float): list of velocity constraints
        amax (list: float): list of acceleration contraints

    Returns:
        # duration (list: floats): list of durations for each actuator movement
        max_dur (float): the largest duration in the list
    '''
    dim = len(q0)
    t_a = [0]*dim
    D_acc = [0]*dim
    D_vmax = [0]*dim
    duration = [0]*dim

    for i in range(dim):
        # Find acceleration time
        t_a[i] = vmax[i]/amax[i]

        # Find acceleration distance
        D_acc[i] = (1/2.0)*amax[i]*(t_a[i]**2)

        # Find remaining distance for constant vel
        D_vmax[i] = abs(qf[i]-q0[i]) - 2*D_acc[i]

        # Sum constant velocity time with 2*(accel time)
        duration[i] = (D_vmax[i]/vmax[i]) + 2*t_a[i]

    max_dur = max(duration)

    return max_dur


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
                 [0]*4])

    coeff = linalg.lstsq(Q, L)[0].transpose()
    return coeff


def sine_traj(q0, qf, v0, vf, j_max=[10]*4):
    '''Function to compute the trajectory parameters using sinusoidal templates.

    Generates the maximum velocity, minimum distance traversed, maximum
    acceleration and time taken to move from q0,v0,a0 to qf,vf,af

    Args:
        q0, qf -- Initial and final position
        v0, vf -- Initial and final velocity
        a0, af -- Initial and final acceleration    (REMOVED)
        j_max -- Maximum jerk of the system
        active -- Active trajectory check N/A       (REMOVED)

    Returns:
        dt -- time taken to reach maximum acceleration
        amax -- maximum possible acceleration
        tf -- Computed final time.
        Dmin -- Minimum distance traveled during acceleration
        vmax -- Maximum velocity in during the traverse

    Comments:
        Also requires the function sine_func.m file to compute the
        temporal points.
    '''

    # Initiate jerk, position, velocity and acceleration of each actuator.
    # j_max = [10]*4
    # q0s = q0
    # v0s = v0
    # a0s = a0

    # Preallocate lists
    vmax = [0]*4
    amax = [0]*4
    Dmin = [0]*4
    dt = [0]*4
    tf = [0]*4

    print 'In sine_traj: ', q0, qf, v0, vf, j_max, '\n'

    # Generating trajectories for each actuator
    for j in range(len(q0)):
        # Compute time taken for acceleration (dt), maximum acceleration
        # achieved (amax), and distance traveled during acceleration (Dmin)
        dt[j] = math.sqrt((abs(vf[j] - v0[j])*np.pi)/(2*j_max[j]))
        amax[j] = (vf[j] - v0[j])/dt[j]
        Dmin[j] = amax[j]*dt[j]*dt[j] + 2*v0[j]*dt[j]
        if abs(2*Dmin[j]) > abs(qf[j]-q0[j]):
            # If distance (Dmin) is more than the
            # total desired actuator travel (q_f-q_0) and if initial velocity
            # is zero compute faster trajectories.
            if v0[j] == 0:
                dt[j] = abs((np.pi*(qf[j]-q0[j]))/(2*j_max[j]*(4-(1/(2*np.pi*np.pi)))))**(1/3.0)
                amax[j] = np.sign(qf[j] - q0[j])*(2*j_max[j]*dt[j])/np.pi
                #(vf-v0)/dt;%(qf-q0)/(2*dt*dt)
                Dmin[j] = amax[j]*dt[j]*dt[j] + 2*v0[j]*dt[j]
                vmax[j] = amax[j]*dt[j]
                # amax*dt*dt+2*v0*dt
            else:
                # reqs implicit solving
                print('Implicit solve not here yet')
        else:
            # If Dmin is within the desired traverse range then compute the
            # maximum velocity within the task.
            vmax[j] = vf[j] - v0[j]

        # Compute the time required to finish the travel.
        tf[j] = 4*dt[j] + (abs(qf[j]-q0[j])-abs(2*Dmin[j]))/abs(vmax[j])

    return dt, amax, tf, Dmin, vmax


def sine_func(t, dt, tf, amax, vmax, q0, v0, a0, qf, p_dprev, Dmin):
    '''Missing info. From sine_func.m from DASLAB group

    Args:
        t (float): time
        dt (float): time to reach max acceleration?
        tf (float): final time
        amax (float): maximum possible acceleration
        vmax (float): maximum possible velocity
        q0 (float): initial position
        v0 (float): initial velocity
        a0 (float): initial acceleration
        qf (float): final position
        p_dprev (float): previous position
        step (float): the hell if I know (REMOVED)
        Dmin (float): minimum distance traveled during acceleration

    Returns:
        p_d (float): position at time t
        v_d (float): velocity at time t
        a_d (float): acceleration at time t
        j_d (float): jerk at time t
    '''
    #function [p_d,v_d,a_d,j_d] = sine_func(t,dt,tf,amax,vmax,q0,v0,a0,qf,p_dprev,step,Dmin)

    # Redefining the maximum jerk.
    # j_max = [10]*4

    # Comment this if using precomputed vmax from sine_traj(..)
    # vmax = np.multiply(amax, dt)  # Define maximum velocity based on maximum acceleration

    # Compute the position, velocity, acceleration and jerk at time 't'.
    if t < (2*dt):
        p_d = (amax/2)*(((dt*dt)/(np.pi*np.pi))*np.cos((np.pi*t)/dt)+(t*t)/2)-(amax*dt*dt)/(2*np.pi*np.pi) + q0
        v_d = (amax/2)*(-(dt/np.pi)*np.sin((np.pi*t)/dt)+t)
        a_d = (amax/2)*(-np.cos((np.pi*t)/dt) + 1)
        j_d = ((amax*np.pi)/(2*dt))*np.sin((np.pi*t)/dt)
    elif t < (tf - 2*dt):
        # p_d = p_dprev + vmax*step
        p_d = ((amax/2)*(((dt*dt)/(np.pi*np.pi))*np.cos((np.pi*2*dt)/dt)+((2*dt)**2)/2)-(amax*dt*dt)/(2*np.pi*np.pi) + q0) + (t-2*dt)*vmax
        v_d = vmax
        a_d = 0
        j_d = 0
    else:
        vmax = amax*dt
        amax = -amax
        t = t - (tf - 2*dt)
        p_d = (amax/2)*(((dt*dt)/(np.pi*np.pi))*np.cos((np.pi*t)/dt)+(t*t)/2)-(amax*dt*dt)/(2*np.pi*np.pi) + (qf - Dmin) + vmax*t
        if abs(p_d-p_dprev) > 0.05:
            stop = True
        v_d = (amax/2)*(-(dt/np.pi)*np.sin((np.pi*t)/dt) + t) + vmax
        a_d = (amax/2)*(-np.cos((np.pi*t)/dt) + 1)
        j_d = ((amax*np.pi)/(2*dt))*np.sin((np.pi*t)/dt)

    return p_d, v_d, a_d, j_d


# Vectorize the previous function
sine_func_v = np.vectorize(sine_func)


def forward_kin_v(exc, sw, bm, sk, bk, bias=0):
    '''This func is the same as 'forward_kin' in this module but is easily vectorized.

    Note: ported to Python from MATLAB "fwd_kin.m", assumed options = [0, 0]

    Args:
        exc (dict): a dict of the excavator physical parameters
        sw (float): the swing angle
        bm (floats): boom displacement in cm
        sk      ^^
        bk      ^^
        bias (float): positive z bias on output, to adjust weird base frame

    Returns:
        eef (list: float): the position of the end-effector (EEF) in (x, y, z - base frame) and the angle of the bucket (axis x4 w.r.t. x1(0?) ground axis)
    '''
    # Assign the base swing angle
    t1 = sw

    # Define lengths
    a1 = exc['a1']
    a2 = exc['a2']
    a3 = exc['a3']
    a4 = exc['a4']

    # Compute or Get joint angles
    # Boom angle
    r_c1 = bm + exc['r_cyl1']
    a_a1b = np.arccos((exc['r_o1b']**2 + exc['r_o1a']**2 - r_c1**2)/(2 * exc['r_o1b']*exc['r_o1a']))
    t2 = a_a1b - exc['a_b12'] - exc['a_a1x1']

    # Stick angle
    r_c2 = sk + exc['r_cyl2']
    a_c2d = np.arccos((exc['r_o2c']**2 + exc['r_o2d']**2 - r_c2**2)/(2 * exc['r_o2c'] * exc['r_o2d']))
    t3 = 3 * np.pi - exc['a_12c'] - a_c2d - exc['a_d23']

    # Bucket angle
    r_c3 = bk + exc['r_cyl3']
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
    # T01 = np.array([[c1, 0, s1, a1 * c1],
    #                 [s1, 0, -c1, a1 * s1],
    #                 [0, 1, 0, 0],
    #                 [0, 0, 0, 1]])

    # T02 = np.array([[c1*c2, -c1*s2, s1, c1*(a2*c2+a1)],
    #                 [s1*c2, -s1*s2, -c1, s1*(a2*c2+a1)],
    #                 [s2, c2, 0, a2*s2],
    #                 [0, 0, 0, 1]])

    # T03 = np.array([[c1*c23, -c1*s23, s1, c1*(a3*c23+a2*c2+a1)],
    #                 [s1*c23, -s1*s23, -c1, s1*(a3*c23+a2*c2+a1)],
    #                 [s23, c23, 0, a3*s23+a2*s2],
    #                 [0, 0, 0, 1]])

    P04 = np.array([[c1*(a4*c234+a3*c23+a2*c2+a1)],
                    [s1*(a4*c234+a3*c23+a2*c2+a1)],
                    [(a4*s234+a3*s23+a2*s2)],
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

    return eef[0], eef[1], eef[2] + bias


def draw_exc(ax, state, lw=2, lock_axes=True, rotate=True, gnd_offset=17.1):
    '''Draws the excavator on 3D xyz axes with lines for each linkage
        uses random colors for each drawing

    Args:
        ax (matplotlib.Axis):  the axis to draw on
        state (array-like): 1-D state vector with length 4 (bm, sk, bk, sw)
        lw (int): linewidth for matplotlib

    Returns:
         none
    '''
    t1 = state[3]
    # Define lengths
    a1 = exc['a1']
    a2 = exc['a2']
    a3 = exc['a3']
    a4 = exc['a4']
    # Compute or Get joint angles
    # Boom angle
    r_c1 = state[0] + exc['r_cyl1']
    a_a1b = np.arccos((exc['r_o1b']**2 + exc['r_o1a']**2 - r_c1**2)/(2 * exc['r_o1b']*exc['r_o1a']))
    t2 = a_a1b - exc['a_b12'] - exc['a_a1x1']

    # Stick angle
    r_c2 = state[1] + exc['r_cyl2']
    a_c2d = np.arccos((exc['r_o2c']**2 + exc['r_o2d']**2 - r_c2**2)/(2 * exc['r_o2c'] * exc['r_o2d']))
    t3 = 3 * np.pi - exc['a_12c'] - a_c2d - exc['a_d23']

    # Bucket angle
    r_c3 = state[2] + exc['r_cyl3']
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

    gos = gnd_offset

    gnd = np.array([0]*3)

    base = np.array([0, 0, gos])

    o1 = np.array([a1*c1, a1*s1, gos])

    o2 = np.array([(a2*c2 + a1)*c1,
                  (a2*c2 + a1)*s1,
                  a2*s2 + gos])

    o3 = np.array([(a2*c2 + a3*c23 + a1)*c1,
                  (a2*c2 + a3*c23 + a1)*s1,
                  gos + a2*s2 + a3*s23])

    o4 = np.array([(a4*c234 + a3*c23 + a2*c2 + a1)*c1,
                   (a4*c234 + a3*c23 + a2*c2 + a1)*s1,
                   gos + a2*s2 + a3*s23 + a4*s234])

    l0 = zip(gnd, base)
    l1 = zip(base, o1)
    l2 = zip(o1, o2)
    l3 = zip(o2, o3)
    l4 = zip(o3, o4)

    # color = np.random.rand(3)

    for line in [l0, l1, l2, l3, l4]:
        ax.plot(line[0], line[1], line[2], '-o', zdir='z', linewidth=lw, c='y', zorder=0)

    if lock_axes:
        ax.set_xlim3d([0, 80])
        ax.set_ylim3d([0, 80])
        ax.set_zlim3d([0, 50])

    if rotate:
        ax.view_init(azim=-137, elev=35)

    return
