#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */

# ******************************************************

# Chris Glomb
# 
# 

# ******************************************************

import sys
import time
from ctypes import *
import cv2
import socket
import numpy as np
import serial
from struct import *

cap = cv2.VideoCapture(0)
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
host ="192.168.2.13" # IP of device to display the image
port = 5005
buf =1024
addr = (host,port)

ser1 = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

imgX = 160
imgY = 120

cx = imgX/2
cy = imgY/2
fov = 60 # in degrees

er = np.ones((4,4),np.uint8)
di = np.ones((2,2),np.uint8)

red_lowerLim = np.array((0.,50.,50.))
red_upperLim = np.array((10.,255.,255.))

green_lowerLim = np.array((50.,50.,40.))
green_upperLim = np.array((70.,255.,255.))

blue_lowerLim = np.array((110.,50.,50.))
blue_upperLim = np.array((130.,255.,255.))


if(sys.argv[1] == 'green'):
    lowerLim = green_lowerLim
    upperLim = green_upperLim
elif(sys.argv[1] == 'blue'):
    lowerLim = blue_lowerLim
    upperLim = blue_upperLim
    
PIDmin = 0
PIDmax = 12

dmxlMin = 50
dmxlMax = 300
    
kp = 20
ki = 0.5
kd = 2

derivator = 0
integrator = 0

P_value = 0
I_value = 0
D_value = 0

direction = 1 # 0 = right, 1 = left 

# calcError - takes in current x and desired x returns error in radians
def calcError(desired, current):
  pixelError = -(desired - current)
  fov_rad = (fov*np.pi)/180.0
  radPerPixX = fov_rad/imgX
  return radPerPixX*pixelError


while True:
    ret, frame = cap.read()
    resized = cv2.resize(frame, (imgX,imgY), interpolation = cv2.INTER_AREA)
    img = cv2.cvtColor(resized,cv2.COLOR_BGR2HSV)
    if(sys.argv[1] == 'red'):
      mask1 = cv2.inRange(img, red_lowerLim, red_upperLim)
      mask2 = cv2.inRange(img, np.array((175.,50.,50.)), np.array((180.,255.,255.)))
      mask = cv2.bitwise_or(mask1,mask2)
    else:
      mask = cv2.inRange(img, lowerLim, upperLim)
    erosion = cv2.erode(mask,er,iterations = 1)
    dilation = cv2.dilate(erosion,di,iterations = 1)
    result = cv2.bitwise_and(resized, resized, mask = mask)
    
    m = cv2.moments(dilation)

    if(m['m00'] != 0.0): # moment is detected
      x = int(m['m10']/m['m00'])
      y = int(m['m01']/m['m00'])
      error = calcError(cx, x)
      if (error >=0): # save known direction of object
        direction = 0
      else:
        direction = 1
      
      # calculate values for PID control
      P_value = kp * error
      D_value = kd * (error - derivator)
      derivator = error
      integrator = integrator + error
      if(integrator > 5):
        integrator = 5
      elif(integrator < -5):
        integrator = -5
      I_value = ki * integrator
      
      PID = P_value + I_value + D_value
      if(PID < 0):
        dmxl = -((((-PID - PIDmin)*(dmxlMax - dmxlMin))/(PIDmax - PIDmin)) + dmxlMin)
      else:
        dmxl = (((PID - PIDmin)*(dmxlMax - dmxlMin))/(PIDmax - PIDmin)) + dmxlMin
      if(dmxl > -60 and dmxl < 60):
        dmxl = 0
      if ser1.isOpen():
        data = pack('hh', dmxl, -dmxl)
        ser1.write(data)  #Send data packet
      
      print "error = ", error, ", PID = ", PID, ", dmxl = ", dmxl
      
    else:
      print "!"
      derivator = 0
      integrator = 0
      if(direction == 0):
        dmxl = 200
      else:
        dmxl = -200

      if ser1.isOpen():
        data = pack('hh', dmxl, -dmxl)
        ser1.write(data)
      
    if(m['m00'] != 0.0):
      cv2.circle(result, (x,y), 2, (0,255,255),-1) # small yellow circle marks center of detected object
    cv2.circle(result, (cx,cy), 2, (0,255,0),-1) # small green circle marks center of screen

    # send what the robot sees to the user
    cv2.imwrite("roboImage.jpg", result)
    f=open ("roboImage.jpg", "rb") 
    data = f.read(buf)
    while (data):
      if(sock.sendto(data,addr)):
        data = f.read(buf)
    f.close()
    sock.sendto("end image",addr)


cv2.destroyAllWindows()










