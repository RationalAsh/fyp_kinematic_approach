#!/usr/bin/bash

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import pdist, squareform, cdist



def incircle(x, r):
    if np.linalg.norm(x) < r:
        return 1.
    else:
        return 0

def knn(X, t, x):
    dists = np.array([np.linalg.norm(x-xi) for xi in X])
    
    

#Define the dataset matrix 
X = np.random.rand(1000, 2) - 0.5
t = np.array([incircle(x, 0.2) for x in X])

print (X.shape, t.shape)

#Try k-NN?
#Get pairwise distances
#pwdist = pdist(

plt.scatter(X[:,0], X[:,1], c=t, cmap='gray')
plt.show()

