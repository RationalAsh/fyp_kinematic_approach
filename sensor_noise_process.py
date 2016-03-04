#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt

X = np.genfromtxt('sensor_noise.txt', delimiter='\t', dtype=float, max_rows=5000) 
