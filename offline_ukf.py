#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
from quaternions import *
from numpy.linalg import norm, inv
import time
#from visual import *

MAX_A = 2*9.81
MAX_G = 250.0*np.pi/180.0
#Parameters for visualization
L, W, H = 1.618, 1.0, 0.2
DT = 0.003

def trap_step(w_c, w_p, dt=DT):
    return (w_c + w_p)*DT/(2)

def F(x, dt=DT):
    '''Function that defines the non-linear State transition
    takes x_k and returns x_k+1'''
    #The quaternion
    qk = x[:4]
    #The angular velocities
    wk = x[4:]

    #Calculate qdelta
    a_delta = norm(wk)*dt
    if norm(wk) != 0:
        e_delta = wk/norm(wk)
        q_delta = axis_angle2quat(e_delta, a_delta)
        #if(wk[2]>0.1):
        #    print(q_delta)
        #Calculate q_new
        q_new = qmul(qk, q_delta)
    elif norm(wk) == 0:
        #Calculate q_new
        q_new = qk    

    #print(str(q_new.shape)+ str(wk.shape))
    #Return the new state vector
    return np.hstack((q_new, wk))

def H(x):
    '''Function that defines the non-linear measurement
    model. Takes x_k and returns z_k'''
    #The quaternion
    qk = x[:4]
    #The angular velocities
    wk = x[4:]

    #Handle the accelerometer model
    g = np.array([0.,0.,0.,-MAX_A/2])
    g_obs = qrot(g,qinv(qk))

    return np.hstack((g_obs, wk))

def compute_mean(Yi):
    '''Function to compute the mean of the set Yi because
    taking a barycentric (normal) mean does not give correct
    results'''
    wmean = np.mean(Yi[:,4:], axis=0)

    #Array for errors
    ei = np.zeros((Yi.shape[0],4))
    ebi = np.zeros((Yi.shape[0],3))

    #Just the quaternions
    Yiq = Yi[:,:4]

    #Initial guess of the mean
    qm = np.array([1.0, 0., 0., 0.])

    #Find error quaternions
    for i in range(len(Yiq)):
        ei[i,:] = qmul(Yiq[i,:], qinv(qm))
    #Find the error vectors
    for i in range(len(Yiq)):
        ebi[i,:] = ei[i,1:]/(np.sin(np.arccos(ei[i,1])))
    #Find the average of the rotation error vectors
    eb = np.mean(ebi, axis=0)
    
    while norm(eb) > 0.0001:
        #Find error quaternions
        for i in range(len(Yiq)):
            ei[i,:] = qmul(Yiq[i,:], qinv(qm))
        #Find the error vectors
        for i in range(len(Yiq)):
            ebi[i,:] = ei[i,1:]/(np.sin(np.arccos(ei[i,1])))
        #Find the average of the rotation error vectors
        eb = np.mean(ebi, axis=0)
        e = np.hstack(([np.cos(norm(eb)/2)], eb*np.sin(norm(eb)/2)/norm(eb)))
        qm = qmul(e,qm)

    return np.hstack((qm, wmean)), ebi

def compute_xi(wi, xk):
    '''Function to compute a disturbed state vector'''
    ww = wi[3:]
    wq = wi[:3]
    ang = norm(wq)
    c = np.cos(ang/2)
    s = np.sin(ang/2)
    if ang != 0:
        ax = wq/ang
        qw = np.hstack((c, ax*s))
        qd = qmul(xk[:4], qw)
    else:
        qd = xk[:4]

    return np.hstack((qd, xk[4:]+ww))

    

DATA = np.loadtxt('sample_data.txt', delimiter='\t')
#Convert to SI units
DATA[:,:3] = DATA[:,:3]*-MAX_A/32786.0
DATA[:,3:6] = DATA[:,3:6]*MAX_G/32786.0

#Get calibration data
CAL_DATA = DATA[:450,:6]
X = DATA[1000:,:6]

#Find the mean and use this to offset the rest of the data
m = np.mean(CAL_DATA, axis=0)
ms = np.tile(np.hstack((np.zeros(3), m[3:])), (X.shape[0], 1))
A = CAL_DATA - np.tile(m, (CAL_DATA.shape[0], 1))
plt.plot(A)
#Covariance matrix
#C = np.dot(A.T, A)
C = np.cov(CAL_DATA, rowvar=0)

X = X - ms

#Plot the sensor values just for fun
plt.figure()
plt.plot(X[:,0], label='ax')
plt.plot(X[:,1], label='ay')
plt.plot(X[:,2], label='az')
plt.plot(X[:,3], label='gx')
plt.plot(X[:,4], label='gy')
plt.plot(X[:,5], label='gz')
plt.xlabel('sample')
plt.ylabel('sensor value')
plt.title('Raw Noisy Sensor Values')
plt.legend()
plt.show()


#We'll be working with X for the filter
#First set the initial values
xk = np.array([1.,0.,0.,0.,0.,0.,0.])
Pk = np.identity(6)
Q = 0.1*np.identity(6)
R = C

#Filtered state vectors
kal_pose = np.zeros((X.shape[0], 7))
#For the Y
Xi = np.zeros((12, 7))
Yi = np.zeros((12, 7))
Zi = np.zeros((12, 6))
ctr = 0

#Stuff for the visualization
#scene = display(title='Quadruped Orientation Visualization', x=0, y=0, width=600, 
#                height=600, center=(0,0,0), background=(0,0,0))
#scene.select()
#The box that represents the body of the quadruped
#bod = box(pos=(0,0,0), length=L, width=W, height=H, axis=(0,-1,0), up=(0,0,1))#, color=(0,1,0))
#qdisplay = label(pos = (-2,0.8,0), xoffset=1, text='Quaternion', height=10, border=6, font='sans')
#Open file to store the data
f = open('ukfiltered_data.txt', 'w+')

zrot = 0.
wz_prev = 0
t = 0
#t_prev = DATA[1000,6]

#Better implementation
for i in range(len(X)):
    st_time = time.time()
    #Get the most recent measurement
    z_k = X[i,:]

    #Estimating the yaw
    #t = DATA[i+1000,6]
    wk = xk[4:]
    qk = xk[:4]
    wz = np.dot(qrot([0,0,1], qk), wk)
    #zrot = zrot + (wz+wz_prev)*(t - t_prev)/2000
    zrot = zrot + (wz+wz_prev)*(DT)/2
    qwz = np.array([np.cos(0.5*zrot),0,0,np.sin(0.5*zrot)])

    wz_prev = wz
    #t_prev = t

    #Compute the set Wi of disturbances in state vectors
    S = np.linalg.cholesky(Pk + Q)
    Wi = np.hstack((np.sqrt(2.0*Pk.shape[0])*S, -np.sqrt(2.0*Pk.shape[0])*S)).T

    #Compute the set Xi of disturbed state vectors
    for j in range(len(Wi)):
        wi = Wi[j,:]        
        Xi[j,:] = compute_xi(wi, xk)    

    #Compute the set of transformed Sigma points
    for j in range(len(Xi)):
        #print(F(Xi[j,:]).shape)
        Yi[j,:] = F(Xi[j,:])

    #Compute the mean and variance of the transformed sigma points
    xk_ap, ebi = compute_mean(Yi)
    Wpi = np.hstack((ebi, Yi[:,4:] - np.tile(xk_ap[4:], (Yi.shape[0],1))))
    #Pk_ap = np.dot(Wpi.T, Wpi)
    Pk_ap = np.cov(Wpi, rowvar=0)

    #Find the set Zi of projected measurement vectors
    for j in range(len(Xi)):
        Zi[j,:] = H(Yi[j,:])
    
    #Finding the measurement estimate covariance and mean
    zk_ap = np.mean(Zi, axis=0)
    Pzz = np.cov(Zi, rowvar=0)
    #Cross covariance term
    Pxz = (1/12.)*np.dot(Wpi.T, Zi - np.tile(zk_ap, (Zi.shape[0], 1)))
    #Innovation Covariance
    Pvv = Pzz + R

    #Calculate the Kalmann gain
    Kk = np.dot(Pxz, inv(Pvv))
    #Calculate innovation
    vk = z_k - zk_ap

    #Get a posteriori
    delta = np.dot(Kk, vk)
    delta_a = norm(delta[:3])
    delta_ax = delta[:3]/delta_a
    q_delta = np.hstack((np.cos(delta_a/2), np.sin(delta_a/2)*delta_ax))
    xk[4:] = xk[4:] + delta[3:]
    xk[:4] = qmul(xk[:4], q_delta)
    Pk = Pk_ap - np.dot(np.dot(Kk, Pvv), Kk.T)
    
    #Store it in an array
    kal_pose[i,:] = xk
    kal_pose[i,:4] = qmul(qwz, kal_pose[i,:4])
    end_time = time.time()

    #print(end_time - st_time)
    print("Count: "+str(ctr)+", angle: "+str(180*2*np.arccos(kal_pose[i,0])/np.pi))
    f.write(str(kal_pose[i,0])+','+str(kal_pose[i,1])+','+str(kal_pose[i,2])+','+str(kal_pose[i,3])+'\n')

    ctr += 1
    #print(ctr)
f.close()


# #Naive implementation
# for i in range(len(X)):
#     row = X[i,:]
#     S = np.linalg.cholesky(Pk + Q)
#     Wi = np.hstack((np.sqrt(2.0*Pk.shape[0])*S,-np.sqrt(2.0*Pk.shape[0])*S)).T
#     Xi = np.tile(xk, (Wi.shape[0],1)) + Wi 

#     for i in range(len(Xi)):
#         Yi[i,:] = F(Xi[i,:])

#     xk_ap = np.mean(Yi,axis=0)
#     Pk_ap = np.cov(Yi, rowvar=0)

#     for i in range(len(Xi)):
#         Zi[i,:] = H(Xi[i,:])

#     zk_ap = np.mean(Zi, axis=0)
#     Pzz = np.cov(Zi, rowvar=0)
#     Pvv = Pzz + R

#     Pxz = np.dot((Yi - np.tile(xk_ap, (Yi.shape[0], 1))).T, Zi - np.tile(zk_ap, (Zi.shape[0], 1)))

#     Kk = np.dot(Pxz, inv(Pvv))
#     vk = row - zk_ap
#     x_k = xk_ap + np.dot(Kk, vk)
#     kal_pose[i,:] = x_k
#     Pk = Pk_ap - np.dot(np.dot(Kk, Pvv), Kk.T)



#Kalman filter loop
#for row in X:
    #Cholesky decompose P + Q
#    S = np.linalg.cholesky(P0 + Q)
#    W = np.hstack((np.sqrt(2.0*P0.shape[0])*S,-np.sqrt(2.0*P0.shape[0])*S)).T

    
    



