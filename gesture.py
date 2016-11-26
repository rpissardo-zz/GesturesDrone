import cv2 as opencv
import numpy as np
import math
import serial
#arduino_serial_port = serial.Serial('/dev/ttyACM0', 9600)
capture_camera = opencv.VideoCapture(0)
count = 0
flag = [0,0,0,0,0]
while(capture_camera.isOpened()):
    ret, image = capture_camera.read()
    opencv.rectangle(image,(350,350),(100,100),(0,255,0),0)
    cropped_image = image[100:350, 100:350]
    grey = opencv.cvtColor(cropped_image, opencv.COLOR_BGR2GRAY)
    value = (35, 35)
    blurred = opencv.GaussianBlur(grey, value, 0)
    _, thresh1 = opencv.threshold(blurred, 127, 255,
                               opencv.THRESH_BINARY_INV+opencv.THRESH_OTSU)
    opencv.imshow('Thresholded', thresh1)
    version = []
    version = opencv.__version__.split('.')

    if version[0] is '3':
        image, contours, hierarchy = opencv.findContours(thresh1.copy(), \
               opencv.RETR_TREE, opencv.CHAIN_APPROX_NONE)
    elif version[0] is '2':
        contours, hierarchy = opencv.findContours(thresh1.copy(),opencv.RETR_TREE, \
               opencv.CHAIN_APPROX_NONE)

    cnt = max(contours, key = lambda x: opencv.contourArea(x))
    
    x,y,w,h = opencv.boundingRect(cnt)
    hull = opencv.convexHull(cnt)
    drawing = np.zeros(cropped_image.shape,np.uint8)
    hull = opencv.convexHull(cnt,returnPoints = False)
    defects = opencv.convexityDefects(cnt,hull)
    count_defects = 0
    for i in range(defects.shape[0]):
        s,e,f,d = defects[i,0]
        start = tuple(cnt[s][0])
        end = tuple(cnt[e][0])
        far = tuple(cnt[f][0])
        a = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        b = math.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
        c = math.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)
        angle = math.acos((b**2 + c**2 - a**2)/(2*b*c)) * 57
        if angle <= 90:
            count_defects += 1
            opencv.circle(cropped_image,far,1,[0,0,255],-1)
        opencv.line(cropped_image,start,end,[0,255,0],2) 
    if count_defects == 1 and flag[0]==0:
        flag = [1,0,0,0,0]
        #arduino_serial_port.write('70p')
    elif count_defects == 2 and flag[1]==0:
        flag = [0,1,0,0,0]
        #arduino_serial_port.write('120p')
    elif count_defects == 3 and flag[2]==0:
        flag = [0,0,1,0,0]
        #arduino_serial_port.write('120r')
    elif count_defects == 4 and flag[3]==0:
        flag = [0,0,0,1,0]
        #arduino_serial_port.write('70r')
    elif count_defects == 0 and flag[4]==0:
        flag = [0,0,0,0,1]
        #arduino_serial_port.write('90p90r')

    if flag[0] == 1:
        opencv.putText(image, "CAMERA UP", (5,50), opencv.FONT_HERSHEY_SIMPLEX, 1, 2)
    elif flag[1] == 1:
        opencv.putText(image, "CAMERA DOWN", (5,50), opencv.FONT_HERSHEY_SIMPLEX, 1, 2)
    elif flag[2] == 1:
        opencv.putText(image,"CAMERA LEFT", (50,50), opencv.FONT_HERSHEY_SIMPLEX, 2, 2)
    elif flag[3] == 1:
        opencv.putText(image,"CAMERA RIGHT", (50,50), opencv.FONT_HERSHEY_SIMPLEX, 2, 2)
    elif flag[4] == 1:
        opencv.putText(image,"CAMERA CENTER", (50,50),opencv.FONT_HERSHEY_SIMPLEX, 2, 2)
    opencv.imshow('Gesture', image)
    all_img = np.hstack((drawing, cropped_image))
    opencv.imshow('Contours', all_img)
    k = opencv.waitKey(10)
    if k == 27:
        break
