#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import time
import cv2

def identity_tf():
    '''Returns the identity transformation'''
    return np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])

def translation_tf(displacement):
    '''Returns a transformation matrix that is
    equivalent to a displacement'''
    return np.array([[1, 0, 0, displacement[0]],
                     [0, 1, 0, displacement[1]],
                     [0, 0, 1, displacement[2]],
                     [0, 0, 0, 1]])

def rotation_tf(angle, axis):
    '''Returns a transformation matrix that is
    equivalent to a rotation of an angle around
    the specified axis. Angle is in radians'''
    ax = axis/np.linalg.norm(axis)
    c = np.cos(angle)
    s = np.sin(angle)
    t = 1 - c
    x = ax[0]
    y = ax[1]
    z = ax[2]

    return np.array([[t*x*x + c, t*x*y - z*s, t*x*z + y*s, 0],
                     [t*x*y + z*s, t*y*y + c, t*y*z - x*s, 0],
                     [t*x*z - y*s, t*y*z + x*s, t*z*z + c, 0],
                     [0, 0, 0, 1]])

def composition(tf_list):
    '''Returns the composition of a list of transformations'''
    tf_final = identity_tf()
    for tf in tf_list:
        tf_final = np.dot(tf_final, tf)

    return tf_final


w, l, d, h = 1.0, 1.618, 0.05, 0.2
#l = [1.0]*9
l1, l2, l3, l4 = 1.0, 1.0, 1.0, 1.0
l5, l6, l7, l8 = 1.0, 1.0, 1.0, 1.0
m = [3.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
th0 = [0.]*9

# def T_LSJ_0(th):
#     return composition(translation_tf([-l/2+d, -w/2-d,0]),
#                        rotation_tf(th[1], [0,-1,0]))
# def T_1_LSJ(th):
#     return translation_tf([0,0,-l1/2])

# def T_LEJ_1(th):
#     return composition(translation_tf([0,0,-l1/2]), 
#                        rotation_tf(th[2], [0,-1,0]))
#def T_2_LEJ(th

def cogf(th):
    #Transformation frames for the front left limb
    T_LSJ_0 = composition([translation_tf([-l/2+d, -w/2-d, 0]), 
                           rotation_tf(th[1], [0,-1,0])])
    T_1_LSJ = translation_tf([0,0,-l1/2])

    T_LEJ_1 = composition([translation_tf( [0,0,-l1/2] ), 
                           rotation_tf(th[2], [0,-1,0])])
    T_2_LEJ = translation_tf( [0,0,-l2/2] )
    T_LH_2  = translation_tf( [0,0,-l2/2] )

    T10 = composition([T_LSJ_0, T_1_LSJ])
    T21 = composition([T_LEJ_1, T_2_LEJ])
    T20 = composition([T10, T21])

    #Transformation frames for the front right limb
    T_RSJ_0 = composition([translation_tf([-l/2+d,w/2+d,0]),
                           rotation_tf(th[3], [0,1,0])])
    T_3_RSJ = translation_tf([0,0,-l3/2])
    T_REJ_3 = composition([translation_tf([0,0,-l3/2]),
                            rotation_tf(th[4], [0,1,0])])
    T_4_REJ = translation_tf([0,0,-l4/2])
    T_RH_4  = translation_tf([0,0,-l4/2])

    T30 = composition([T_RSJ_0, T_3_RSJ])
    T43 = composition([T_REJ_3, T_4_REJ])
    T40 = composition([T30, T43])

    #Transformation frames for the rear right limb
    T_RHJ_0 = composition([translation_tf([l/2-d,w/2+d,0]),
                           rotation_tf(th[5], [0,1,0])])
    T_5_RHJ = translation_tf([0,0,-l5/2])
    T_RKJ_5 = composition([translation_tf([0,0,-l5/2]),
                            rotation_tf(th[6], [0,1,0])])
    T_6_RKJ = translation_tf([0,0,-l6/2])
    T_RF_6 = translation_tf([0,0,-l6/2])

    T50 = composition([T_RHJ_0, T_5_RHJ])
    T65 = composition([T_RKJ_5, T_6_RKJ])
    T60 = composition([T50, T65])

    #Transformation frames for the rear left limb
    T_LHJ_0 = composition([translation_tf([l/2-d,-w/2-d,0]),
                           rotation_tf(th[7], [0,-1,0])])
    T_7_LHJ = translation_tf([0,0,-l7/2])
    T_LKJ_7 = composition([translation_tf([0,0,-l7/2]),
                            rotation_tf(th[8], [0,-1,0])])
    T_8_LKJ = translation_tf([0,0,-l8/2])
    T_LF_8  = translation_tf([0,0,-l8/2])

    T70 = composition([T_LHJ_0, T_7_LHJ])
    T87 = composition([T_LKJ_7, T_8_LKJ])
    T80 = composition([T70, T87])

    #Feet
    T_LH_0 = composition([T20, T_LH_2])
    T_RH_0 = composition([T40, T_RH_4])
    T_RF_0 = composition([T60, T_RF_6])
    T_LF_0 = composition([T80, T_LF_8])

    zero = np.array([0,0,0,1])
    total_mass = sum(m)
    cog = (m[1]*np.dot(T10, zero) + m[2]*np.dot(T20, zero) +\
           m[3]*np.dot(T30, zero) + m[4]*np.dot(T40, zero) +\
           m[5]*np.dot(T50, zero) + m[6]*np.dot(T60, zero) +\
           m[7]*np.dot(T70, zero) + m[8]*np.dot(T80, zero))/total_mass
    return cog[:3]

def is_stable_possible(th, foot='LH'):
    rval1 = 0
    rval2 = 0

    #Transformation frames for the front left limb
    T_LSJ_0 = composition([translation_tf([-l/2+d, -w/2-d, 0]), 
                           rotation_tf(th[1], [0,-1,0])])
    T_1_LSJ = translation_tf([0,0,-l1/2])

    T_LEJ_1 = composition([translation_tf( [0,0,-l1/2] ), 
                           rotation_tf(th[2], [0,-1,0])])
    T_2_LEJ = translation_tf( [0,0,-l2/2] )
    T_LH_2  = translation_tf( [0,0,-l2/2] )

    T10 = composition([T_LSJ_0, T_1_LSJ])
    T21 = composition([T_LEJ_1, T_2_LEJ])
    T20 = composition([T10, T21])

    #Transformation frames for the front right limb
    T_RSJ_0 = composition([translation_tf([-l/2+d,w/2+d,0]),
                           rotation_tf(th[3], [0,1,0])])
    T_3_RSJ = translation_tf([0,0,-l3/2])
    T_REJ_3 = composition([translation_tf([0,0,-l3/2]),
                            rotation_tf(th[4], [0,1,0])])
    T_4_REJ = translation_tf([0,0,-l4/2])
    T_RH_4  = translation_tf([0,0,-l4/2])

    T30 = composition([T_RSJ_0, T_3_RSJ])
    T43 = composition([T_REJ_3, T_4_REJ])
    T40 = composition([T30, T43])

    #Transformation frames for the rear right limb
    T_RHJ_0 = composition([translation_tf([l/2-d,w/2+d,0]),
                           rotation_tf(th[5], [0,1,0])])
    T_5_RHJ = translation_tf([0,0,-l5/2])
    T_RKJ_5 = composition([translation_tf([0,0,-l5/2]),
                            rotation_tf(th[6], [0,1,0])])
    T_6_RKJ = translation_tf([0,0,-l6/2])
    T_RF_6 = translation_tf([0,0,-l6/2])

    T50 = composition([T_RHJ_0, T_5_RHJ])
    T65 = composition([T_RKJ_5, T_6_RKJ])
    T60 = composition([T50, T65])

    #Transformation frames for the rear left limb
    T_LHJ_0 = composition([translation_tf([l/2-d,-w/2-d,0]),
                           rotation_tf(th[7], [0,-1,0])])
    T_7_LHJ = translation_tf([0,0,-l7/2])
    T_LKJ_7 = composition([translation_tf([0,0,-l7/2]),
                            rotation_tf(th[8], [0,-1,0])])
    T_8_LKJ = translation_tf([0,0,-l8/2])
    T_LF_8  = translation_tf([0,0,-l8/2])

    T70 = composition([T_LHJ_0, T_7_LHJ])
    T87 = composition([T_LKJ_7, T_8_LKJ])
    T80 = composition([T70, T87])

    #Feet
    T_LH_0 = composition([T20, T_LH_2])
    T_RH_0 = composition([T40, T_RH_4])
    T_RF_0 = composition([T60, T_RF_6])
    T_LF_0 = composition([T80, T_LF_8])

    zero = np.array([0,0,0,1])
    total_mass = sum(m)
    cog = (m[1]*np.dot(T10, zero) + m[2]*np.dot(T20, zero) +\
           m[3]*np.dot(T30, zero) + m[4]*np.dot(T40, zero) +\
           m[5]*np.dot(T50, zero) + m[6]*np.dot(T60, zero) +\
           m[7]*np.dot(T70, zero) + m[8]*np.dot(T80, zero))/total_mass
    cog = cog[:3]
    P1 = np.array([0]*4)
    P2 = np.array([0]*4)
    P3 = np.array([0]*4)
    P_f = np.array([0]*4)
    
    if foot == 'LH':
        P1 = np.dot(T_RH_0, zero)[:3]
        P2 = np.dot(T_RF_0, zero)[:3]
        P3 = np.dot(T_LF_0, zero)[:3]

        P_f = np.dot(T_LH_0, zero)[:3]
        #print 'LH!!'
    elif foot == 'RH':
        P1 = np.dot(T_LH_0, zero)[:3]
        P2 = np.dot(T_RF_0, zero)[:3]
        P3 = np.dot(T_LF_0, zero)[:3]

        P_f = np.dot(T_RH_0, zero)[:3]
        #print 'LH!!'
    elif foot == 'RF':
        P1 = np.dot(T_LH_0, zero)[:3]
        P2 = np.dot(T_RH_0, zero)[:3]
        P3 = np.dot(T_LF_0, zero)[:3]

        P_f = np.dot(T_RF_0, zero)[:3]
        #print 'LH!!'
    elif foot == 'LF':
        P1 = np.dot(T_LH_0, zero)[:3]
        P2 = np.dot(T_RH_0, zero)[:3]
        P3 = np.dot(T_RF_0, zero)[:3]

        P_f = np.dot(T_LF_0, zero)[:3]
        #print 'LH!!'

    norm = np.cross(P3-P2, P3-P1)
    norm = norm/np.linalg.norm(norm)
    P_proj = cog + (np.dot(P1,norm) - np.dot(cog, norm))*norm

    #Condition for foot
    if(np.dot(P_f, norm) > np.dot(P3, norm)):
        rval2 = 1
    #print cog
    #print np.dot(P1,norm)
    #print P_proj

    a1 = np.linalg.norm(np.cross(P3-P_proj, P2-P_proj)) +\
         np.linalg.norm(np.cross(P2-P_proj, P1-P_proj)) +\
         np.linalg.norm(np.cross(P3-P_proj, P1-P_proj))

    a2 = np.linalg.norm(np.cross(P3-P1, P3-P2))

    #print a1, a2


    if(abs(a1-a2) < 0.001):
        rval1 = 1
    else:
        rval1 = 0
    
    return rval1, rval2



if __name__ == "__main__":
    res = 100
    fig = plt.figure()
    stability = np.zeros((res,res))    

    print rotation_tf(30*3.14159/180, [0, 1, 0])
    print translation_tf([1, 1, 1])
    print composition([translation_tf([-l/2+d, -w/2 - d, 0]), 
                       rotation_tf(3.14159/6, [0, -1, 0])])
    
    th1 = np.linspace(-np.pi/2, np.pi/2, res)
    th2 = np.linspace(-np.pi/2, np.pi/2, res)

    th1v, th2v = np.meshgrid(th1, th2, indexing='xy')
    
    print "Nom"
    for i in xrange(res):
        for j in xrange(res):
            sval = is_stable_possible([0.0, th1v[j,i], th2v[j,i], -0.4, 0.8, -0.8, 1.6, 0.8, -1.6])
            stability[res-1-j][i] = sval[0] and sval[1]
            cv2.imshow("Stability", stability)
            cv2.waitKey(1)

    plt.imshow(stability)
    plt.show()


