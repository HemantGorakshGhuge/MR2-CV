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

    crop_img = frame[100:480, 0:640]

    gray = cv.cvtColor(crop_img, cv.COLOR_BGR2GRAY)

    blur = cv.GaussianBlur(gray,(5,5),1)

    ret, thresh = cv.threshold(blur,155,255,cv.THRESH_BINARY)
    cv.imshow('thresh', thresh)

    _, contours, _ = cv.findContours(thresh.copy(), 1, cv.CHAIN_APPROX_NONE)


    if len(contours) > 0:
        con = max(contours, key = cv.contourArea)
        M = cv.moments(con)

        if M['m00'] == 0:
            M['m00'] = 0.0000001;

        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        cv.line(crop_img,(cx,0),(cx,480), (255,0,0),1)
        cv.line(crop_img,(0,cy),(640,cy), (255,0,0),1)

        cv.drawContours(crop_img, contours, -1, (0,255,0), 1)
 
        cx_new = (cx/640)*100
        #pi_pwm.ChangeDutyCycle(cx_new)

        print ("cx = " + str(cx))

        if cx >= 0 and cx < 128: # left turn
            gpio.output(a,gpio.LOW)
            gpio.output(b,gpio.LOW)
            gpio.output(c,gpio.LOW)
            gpio.output(d,gpio.HIGH)            

        elif cx >= 128 and cx < 256: # slight left turn
            gpio.output(a,gpio.LOW)
            gpio.output(b,gpio.LOW)
            gpio.output(c,gpio.HIGH)
            gpio.output(d,gpio.LOW)            
        
        elif cx >= 256 and cx < 384: # forward
            gpio.output(a,gpio.LOW)
            gpio.output(b,gpio.LOW)
            gpio.output(c,gpio.HIGH)
            gpio.output(d,gpio.HIGH)            
        
        elif cx >= 384 and cx < 512: # slight right turn
            gpio.output(a,gpio.LOW)
            gpio.output(b,gpio.HIGH)
            gpio.output(c,gpio.LOW)
            gpio.output(d,gpio.LOW)            

        elif cx >= 512 and cx < 640: # right turn
            gpio.output(a,gpio.LOW)
            gpio.output(b,gpio.HIGH)
            gpio.output(c,gpio.LOW)
            gpio.output(d,gpio.HIGH)            

        else:
            gpio.output(a,gpio.LOW)
            gpio.output(b,gpio.LOW)
            gpio.output(c,gpio.LOW)
            gpio.output(d,gpio.LOW)            


    else:
        print ('I don\'t see the line')


    
    cv.imshow('crop_img',crop_img)

    if cv.waitKey(1) & 0x77 == ord('q'):
        break

