#!/usr/bin/python

'''This is a quaternions library that I am developing
on my own for my final year project. It's targeted at
people who want to use quaternions for spatial rotations.'''

import numpy as np
from numpy.linalg import norm


def axis_angle2quat(ax, ang):
    ax = np.array(ax)
    s = np.sin(ang/2)
    ax_norm = np.linalg.norm(ax)
    if np.sum(ax) != 0:
        ax = ax/np.linalg.norm(ax)
    
    return np.array([np.cos(ang/2), ax[0]*s, 
                     ax[1]*s, ax[2]*s])

def quat2axis_angle(q):
    angle = 2*np.arccos(q[0])
    k = np.sqrt(1 - q[0]**2)
    ax = np.array(q[1:])/k
    return ax, angle

def qmul(q1, q2):
    '''Takes quaternions of the form 
    w + xi + yj + zk represented as
    numpy arrays'''
    a1, b1, c1, d1 = q1[0], q1[1], q1[2], q1[3]
    a2, b2, c2, d2 = q2[0], q2[1], q2[2], q2[3]
    
    return np.array([a1*a2 - b1*b2 - c1*c2 - d1*d2,
                     a1*b2 + b1*a2 + c1*d2 - d1*c2,
                     a1*c2 - b1*d2 + c1*a2 + d1*b2,
                     a1*d2 + b1*c2 - c1*b2 + d1*a2])

def qinv(q):
    '''Returns the conjugation of a quaternion'''
    return np.array([q[0],-q[1],-q[2],-q[3]])

def qrot(p, q):
    p = np.array(p)
    q = np.array(q)
    if len(p) == 3:
        p = np.array([0., p[0], p[1], p[2]])

    #Check if q is valid rotation quaternion
    if np.linalg.norm(q) != 1.0:
        #print("Warning! Not unit quaternion")
        q = q/norm(q)
        pass

    qi = qmul(q, p)
    qfinal = qmul(qi, qinv(q))
    
    return qfinal[1:]
