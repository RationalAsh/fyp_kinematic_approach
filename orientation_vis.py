#!/usr/bin/python

#Orientation visualization using quaternions
#From the DMP6 onboard the GY521

from visual import *
import numpy as np
import serial
from serial import SerialException
import time
from quaternions import *
from numpy import array as ar
import threading
from queue import Queue
from collections import deque

PORT = '/dev/ttyUSB0'
#PORT = 'dev/rfcomm0'
BAUD = 115200
L, W, H = 1.618, 1.0, 0.2  

delay = 0.01

def rotate(point, quat, homequat):
    #p1 = qrot(point, qinv(homequat))
    #p2 = qrot(p1, quat)
    
    final_q = qmul(homequat, quat)
    
    return qrot(point, final_q)

def serial_thread(PORT, BAUD, qu):
    sensorvals = ar([0., 0., 0., 0.])
    line = ''
    sport = None
    SFLAG = 0
    global sensor_quat

    while True:
        if (SFLAG == 0):
            try:
                sport = serial.Serial(PORT, BAUD)
                SFLAG = 1
            except:
                SFLAG = 0
        else:
            try:
                line = sport.readline().strip()
                strvals = line.split('\t')
                #print(line)
            except:
                SFLAG = 0
                sport.close()
            try:
                sensorvals = ar([float(i) for i in strvals])
                #print "Serial Thread: "+str(sensorvals)
                qu.append(sensorvals)
            except:
                pass
        #time.sleep(0.01)

#Try to connect to the serial port
try:
    ser = serial.Serial(PORT, BAUD)
    SFLAG = 1
except:
    print "Arduino Not Connected"
    SFLAG = 0


#The box that represents the body of the quadruped
scene = display(title='Quadruped Orientation Visualization', x=0, y=0, width=600, 
                height=600, center=(0,0,0), background=(0,0,0))
scene.select()

bod = box(pos=(0,0,0), length=L, width=W, height=H, axis=(0,-1,0), up=(0,0,1), color=(0,1,0))
#ax = arrow(pos=(0,0,0), axis=(1.,0,0), shaftwidth=0.1)
#ay = arrow(pos=(0,0,0), axis=(0,1.,0), shaftwidth=0.1)
#az = arrow(pos=(0,0,0), axis=(0,0,1.), shaftwidth=0.1) 
qdisplay = label(pos = (-2,0.8,0), xoffset=1, text='Quaternion', height=10, border=6, font='sans')

#Quaternion for orientation
quat_qu = Queue(maxsize=10)
quat_dqu = deque(maxlen=5)

home_quat = ar([1., 0., 0., 0.])

ser_thread = threading.Thread(target=serial_thread, args=(PORT, BAUD, quat_dqu))
ser_thread.setDaemon(True)
ser_thread.start()

#Wait for bit
time.sleep(0.8)

quat = ar([1., 0., 0., 0.])
while True:
    try:
        quat = quat_dqu.popleft()
    except:
        pass

    #ax, ang = quat2axis_angle(quat)
    qdisplay.text = "Quaternion: "+str(quat)
    boxax = rotate([-1*L,0,0], quat, home_quat)
    boxup = rotate([0,0,1], quat, home_quat)
    #print(boxax)
    #print(boxup)
    bod.axis = tuple(boxax)
    bod.up = tuple(boxup)
    rate(100)

    #Press H to set base rotation
    if scene.kb.keys: # event waiting to be processed?
        s = scene.kb.getkey() # get keyboard info
        if s == 'h' or s=='H':
            print('You pressed H!')
            home_quat = quat
