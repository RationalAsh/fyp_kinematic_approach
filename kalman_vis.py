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

#MODES
CALIB = 1
DISP = 2
FILT = 3

PORT = '/dev/ttyUSB0'
#PORT = 'dev/rfcomm0'
BAUD = 115200
L, W, H = 1.618, 1.0, 0.2  

delay = 0.01

MODE = DISP



def serial_thread(PORT, BAUD, qu):
    sensorvals = ar([0., 0., 0., 0.])
    line = ''
    sport = None
    SFLAG = 0

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
qdisplay = label(pos = (-2,0.8,0), xoffset=1, text='Quaternion', height=10, border=6, font='sans')
modetxt = label(pos = (-2,0.8,0), xoffset=1, text='Mode', height=10, border=6, font='sans')

sensor_dqu = deque(maxlen=5)

ser_thread = threading.Thread(target=serial_thread, args=(PORT, BAUD, sensor_dqu))
ser_thread.setDaemon(True)
ser_thread.start()

#Wait for bit
time.sleep(0.8)

sensor_sample = np.zeros(6)


#Initial values of the Kalman Filter

acc = np.zeros(6)
ctr = 0

while True:
    #Get the most recent sensor values
    try:
        sensor_sample = sensor_dqu.popleft()
    except:
        pass

    #Measure for a bit and set the sensor offsets
    if MODE == CALIB:
        

    #Do the Unscented Kalman Filtering Here
    

    #Get the estimate of the current quaterion
    #Use estimate to rotate cube


    #ax, ang = quat2axis_angle(quat)
    qdisplay.text = "Sensors: "+str(sensor_sample)
    rate(100)

    #Press H to set base rotation
    if scene.kb.keys: # event waiting to be processed?
        s = scene.kb.getkey() # get keyboard info
        if s == 'h' or s=='H':
            print('You pressed H!')
        elif s == 'c' or s=='C':
            modetxt.text = 'Mode: '+'Calibration'
            MODE = CALIB
        elif s == 'd' or s=='D':
            modetxt.text = 'Mode: '+'Display'
            MODE = DISP
        elif s == 'f' or s=='F':
            modetxt.text = 'Mode: '+'Filter'
            MODE = FILT
            
