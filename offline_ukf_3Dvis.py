#!/usr/bin/python

from visual import *
import numpy as np
from quaternions import *


#Parameters for the model
L, W, H = 1.618, 1.0, 0.2
RAD2DEG = 180/np.pi


#The scene for visualization
scene = display(title='Quadruped Orientation Visualization', x=0, y=0, width=600, 
                height=600, center=(0,0,0), background=(0,0,0))
scene.select()
#The box that represents the body of the quadruped
bod = box(pos=(0,0,0), length=L, width=W, height=H, axis=(0,-1,0), up=(0,0,1), color=(0,1,0))
qdisplay = label(pos = (-2,0.8,0), xoffset=1, text='Quaternion', height=10, border=6, font='sans')

ctr = 0

QUATS = np.loadtxt('ukfiltered_data.txt', delimiter=',')
#RAW = np.loadtxt('sample_data.txt', delimiter='\t')
yprvals = np.zeros((QUATS.shape[0],3))

for quat in QUATS:
    #Get the next orientation quaternion
    #print(quat)
    ypr = quat2euler(quat)
    yprvals[ctr, :] = ypr
        
    #Update the orientation of the box
    bod.axis = qrot([-L, 0., 0.], quat)
    bod.up = qrot([0., 0., 1], quat)

    #Update the label with the current quaternion
    qdisplay.text = 'Quaternion: '+str(quat)+'\n'+\
                    'Yaw:   '+str(ypr[0]*RAD2DEG)+'\n'+\
                    'Pitch: '+str(ypr[1]*RAD2DEG)+'\n'+\
                    'Roll:  '+str(ypr[2]*RAD2DEG)+'\n'
    ctr += 1
    rate(250)
