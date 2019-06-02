import numpy as np
import cv2 as cv
import RPi.GPIO as gpio
from time import sleep

video_capture = cv.VideoCapture(-1)
video_capture.set(3,640)
video_capture.set(4,480)

#pin = 12
a=31
b=33
c=35
d=37

gpio.setwarnings(False)
gpio.setmode(gpio.BOARD)
#gpio.setup(pin,gpio.OUT)
#pi_pwm = gpio.PWM(pin,1000)
#pi_pwm.start(0)
gpio.setup(a,gpio.OUT)
gpio.setup(b,gpio.OUT)
gpio.setup(c,gpio.OUT)
gpio.setup(d,gpio.OUT)
gpio.output(a,gpio.LOW)
gpio.output(b,gpio.LOW)
gpio.output(c,gpio.LOW)
gpio.output(d,gpio.LOW)
while(True):

    ret, frame = video_capture.read()

    crop_img = frame[0:480, 0:1080]

    hsv = cv.cvtColor(crop_img, cv.COLOR_BGR2HSV)
    
    #calculations:
    #hsv_red = cv2.cvtColor(red,cv2.COLOR_BGR2HSV)
    lower_red = np.array([160, 50, 50])
    upper_red = np.array([180, 255, 255])
    
    mask_red = cv.inRange(hsv, lower_red, upper_red)
    cv.imshow('mask red', mask_red)

    kernel = np.ones((3,3), np.uint8)
    red_line = cv.morphologyEx(mask_red, cv.MORPH_OPEN, kernel)
    cv.imshow('red line', red_line)

    _, contours_red, _= cv.findContours(red_line.copy(), 1, cv.CHAIN_APPROX_NONE)

    if len(contours_red) > 0:

        con_red = max(contours_red, key = cv.contourArea)
        area_red = cv.contourArea(con_red)
        print('area_red = ' + str(area_red))

        if (area_red > 4000):
            M_red = cv.moments(con_red)

            if M_red['m00'] == 0:
                M_red['m00'] == 0.0000001;

            cx_red = int(M_red['m10']/M_red['m00'])
            cy_red = int(M_red['m01']/M_red['m00'])

            cv.line(crop_img, (cx_red,0), (cx_red,480), (255,255,0),2)
            cv.line(crop_img, (0,cy_red), (1080,cy_red), (255,255,0),2)

            cv.drawContours(crop_img, contours_red, -1, (0,255,255), 2)
        
    cv.imshow('crop_img', crop_img)

    if cv.waitKey(1) & 0x77 == ord('q'):
        break

