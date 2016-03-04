#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from numpy.linalg import inv, pinv
import cv2


#All the parameters
w, l, d, h = 1.0, 1.618, 0.05, 0.2
#l = [1.0]*9
l1, l2, l3, l4 = 1.0, 1.0, 1.0, 1.0
l5, l6, l7, l8 = 1.0, 1.0, 1.0, 1.0
m = [3.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
th0 = [0.]*9

def T10(th):
    return np.array([d - 0.5*l + 0.5*l1*np.sin(th[1]),
                     -d - 0.5*w,
                     -0.5*l1*np.cos(th[1])])

def T20(th):
    return np.array([d - 0.5*l + l1*np.sin(th[1]) + 0.5*l2*np.sin(th[1]+th[2]),
                     -d - 0.5*w,
                     -l1*np.cos(th[1]) - 0.5*l2*np.cos(th[1]+th[2])])

def T30(th):
    return np.array([d - 0.5*l - 0.5*l3*np.sin(th[3]),
                     d + 0.5*w,
                     -0.5*l3*np.cos(th[3])])

def T40(th):
    return np.array([d - 0.5*l - l3*np.sin(th[3]) - 0.5*l4*np.sin(th[3]+th[4]), 
                     d + 0.5*w,
                     -l3*np.cos(th[1]) - 0.5*l4*np.cos(th[3]+th[4])])

def T50(th):
    return np.array([-d + 0.5*l - 0.5*l5*np.sin(th[5]),
                     d + 0.5*w,
                     -0.5*l5*np.cos(th[5])])

def T60(th):
    return np.array([-d + 0.5*l - l5*np.sin(th[5]) - 0.5*l6*np.sin(th[5]+th[6]), 
                     d + 0.5*w,
                     -l5*np.cos(th[5]) - 0.5*l6*np.cos(th[5]+th[6])])

def T70(th):
   return np.array([-d + 0.5*l + 0.5*l7*np.sin(th[7]),
                     -d - 0.5*w,
                     -0.5*l7*np.cos(th[7])])

def T80(th):
    return np.array([-d + 0.5*l + l7*np.sin(th[7]) + 0.5*l8*np.sin(th[7]+th[8]), 
                     -d - 0.5*w,
                     -l7*np.cos(th[7]) - 0.5*l8*np.cos(th[7]+th[8])])

def TLH0(th):
    return np.array([d - 0.5*l + l1*np.sin(th[1]) + 0.5*l2*np.sin(th[1]+th[2]),
                     -d - 0.5*w,
                     -l1*np.cos(th[1]) - l2*np.cos(th[1]+th[2])])

def TRH0(th):
    return np.array([d - 0.5*l - l3*np.sin(th[3]) - 0.5*l4*np.sin(th[3]+th[4]), 
                     d + 0.5*w,
                     -l3*np.cos(th[1]) - l4*np.cos(th[3]+th[4])])

def TRF0(th):
    return np.array([-d + 0.5*l - l5*np.sin(th[5]) - 0.5*l6*np.sin(th[5]+th[6]), 
                     d + 0.5*w,
                     -l5*np.cos(th[5]) - l6*np.cos(th[5]+th[6])])

def TLF0(th):
     return np.array([-d + 0.5*l + l7*np.sin(th[7]) + 0.5*l8*np.sin(th[7]+th[8]), 
                     -d - 0.5*w,
                     -l7*np.cos(th[7]) - l8*np.cos(th[7]+th[8])])

def cogf(th):
    return m[1]*T10(th) + m[2]*T20(th) + m[3]*T30(th) +\
        m[4]*T40(th) + m[5]*T50(th) + m[6]*T60(th) +\
        m[7]*T70(th) + m[8]*T80(th)

def cgcheck(cog, P1, P2, P3):
    #Check if CG is in triangle using barycentric
    #Coordinates. Barycentric coords are cool
    U = P3 - P1
    V = P2 - P1
    A = np.vstack((U,V)).T
    pA = pinv(A)
    T = np.dot(A, pA)
    X = np.vstack((cog, P1, P2, P3)).T
    Xt = np.dot(T, X)
    try:
        gamma = np.dot(inv(Xt[:,1:]), Xt[:,0])
    except:
        return 0
    
    if (gamma[0] < 1.0) and (gamma[1] < 1.0) and (gamma[2] < 1.0):
        return 1
    else:
        return 0

def cgcheck2(cog, P1, P2, P3):
    norm = np.cross(P3-P2, P3-P1)
    norm = norm/np.linalg.norm(norm)
    P_proj = cog + (np.dot(P1,norm) - np.dot(cog, norm))*norm

    a1 = np.linalg.norm(np.cross(P3-P_proj, P2-P_proj)) +\
         np.linalg.norm(np.cross(P2-P_proj, P1-P_proj)) +\
         np.linalg.norm(np.cross(P3-P_proj, P1-P_proj))

    a2 = np.linalg.norm(np.cross(P3-P1, P3-P2))
    
    if(abs(a1-a2) < 0.001):
        return 1
    else:
        return 0
    

def isStable(th, active='LH'):
    #Get the COM coordinates
    cog = cogf(th)

    #Get the 3 points that form the stability polygon
    P1 = TLH0(th)
    P2 = TRH0(th)
    P3 = TRF0(th)
    P4 = TLF0(th)

    #Determine if the thing is stable while moving just two angles
    #Depends on which angles are being changed
    
    #Which leg is moving?
    if active == 'LH':
        return cgcheck2(cog, P2, P3, P4)
    elif active == 'RH':
        return cgcheck2(cog, P1, P3, P4)
    elif active == 'RF':
        return cgcheck2(cog, P1, P2, P4)
    elif active == 'LF':
        return cgcheck2(cog, P1, P2, P3)


        
        
if __name__ == "__main__":

    fig = plt.figure()
    stability = np.zeros((256,256))    

    th1 = np.linspace(-np.pi/2, np.pi/2, 256)
    th2 = np.linspace(-np.pi/2, np.pi/2, 256)

    th1v, th2v = np.meshgrid(th1, th2, indexing='xy')
    
    print "Nom"
    for i in xrange(256):
        for j in xrange(256):
            sval = isStable([0.0, th1v[j,i], th2v[j,i], -0.4, 0.8, -0.4, 0.8, 0.4, -0.8])
            stability[255-j][i] = sval
            cv2.imshow("Stability", stability)
            cv2.waitKey(1)

    plt.imshow(stability)
    plt.show()



