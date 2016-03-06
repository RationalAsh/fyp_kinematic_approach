#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt

MAX_A = 2*9.81
MAX_G = 250.0*np.pi/180.0
#Parameters for visualization
L, W, H = 1.618, 1.0, 0.2

#Load the sensor data
DATA = np.loadtxt('sample_data.txt', delimiter='\t')
#Convert to SI units
DATA[:,:3] = DATA[:,:3]*-MAX_A/32786.0
DATA[:,3:6] = DATA[:,3:6]*MAX_G/32786.0

#Get calibration data
CAL_DATA = DATA[:450,:6]
X = DATA[:,:6]

#Find the mean and use this to offset the rest of the data
m = np.mean(CAL_DATA, axis=0)
ms = np.tile(np.hstack((np.zeros(3), m[3:])), (X.shape[0], 1))
X = X - ms

plt.plot(X)
plt.show()

