import numpy as np
import cv2 as cv
import RPi.GPIO as gpio
from time import sleep

video_capture = cv.VideoCapture(-1)
video_capture.set(3,1080)
video_capture.set(4,480)
video_capture.set(5,90)

a=31
b=33
c=35
d=37

gpio.setwarnings(False)
gpio.setmode(gpio.BOARD)

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

    gray = cv.cvtColor(crop_img, cv.COLOR_BGR2GRAY)

    blur = cv.GaussianBlur(gray,(5,5),1)

    ret, thresh = cv.threshold(blur,165,255,cv.THRESH_BINARY)

    kernel = np.ones((3,3), np.uint8)
    white_line = cv.morphologyEx(thresh, cv.MORPH_OPEN, kernel)
    
    cv.imshow('white line', white_line)

    _, contours, _ = cv.findContours(white_line.copy(), 1, cv.CHAIN_APPROX_NONE)

    hsv = cv.cvtColor(crop_img, cv.COLOR_BGR2HSV)
    
    #calculations:
    #red = np.uint8([[[0,0,255 ]]])
    #hsv_red = cv2.cvtColor(red,cv2.COLOR_BGR2HSV)
    #hsv_red = [170,]
    #[hue-10, 50, 50]
    #[hue+10, 255, 255]
    lower_red = np.array([160, 50, 50])
    upper_red = np.array([180, 255, 255])
    
    mask = cv.inRange(hsv, lower_red, upper_red)
    
    if (mask.any()!=0):
        res = cv.bitwise_and(crop_img, crop_img, mask= mask)
        #area_line = cv.contourArea(res)
        #print('area_line = '+ str(area_line))
        gpio.output(a,gpio.HIGH)
        #gpio.output(b,gpio.LOW)
        #gpio.output(c,gpio.LOW)
        #gpio.output(d,gpio.LOW)
        print ('a = HIGH')
        
        cv.imshow('res', res)
    
    else:
        gpio.output(a,gpio.LOW)
        #gpio.output(b,gpio.LOW)
        #gpio.output(c,gpio.LOW)
        #gpio.output(d,gpio.LOW)
        print ('a = LOW')
        #cv.DestroyWindow('res') 
    #cv.imshow('crop_img', crop_img)
    cv.imshow('mask', mask)


    if len(contours) > 0:
        
        con = max(contours, key = cv.contourArea)
        area = cv.contourArea(con)
        print('area = ' + str(area))
        
        if (area > 900):
            M = cv.moments(con)
         
        #blackbox = cv.minAreaRect(contours[0])
        #(x_min, y_min), (w_min, h_min), ang = blackbox

        #if ang < -45 :
         #   ang = 90 + ang
          
        #if w_min < h_min and ang > 0:    
         #   ang = (90-ang)*-1
                      
        #if w_min > h_min and ang < 0:
         #   ang = 90 + ang
            
        #ang = int(ang)
        #print('ang = '+ str(ang))

            if M['m00'] == 0:
                M['m00'] = 0.0000001;

            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            cv.line(crop_img,(cx,0),(cx,480), (255,0,0),1)
            cv.line(crop_img,(0,cy),(1080,cy), (255,0,0),1)

            cv.drawContours(crop_img, contours, -1, (0,255,0), 2)
     
            #cx_new = (cx/640)*100
            #pi_pwm.ChangeDutyCycle(cx_new)

            print ("cx = " + str(cx))

            if cx >= 0 and cx < 301: # left turn
                #gpio.output(a,gpio.LOW)
                gpio.output(b,gpio.LOW)
                gpio.output(c,gpio.LOW)
                gpio.output(d,gpio.HIGH)
                print('left turn')

            elif cx >= 301 and cx < 387: # slight left turn
                #gpio.output(a,gpio.LOW)
                gpio.output(b,gpio.LOW)
                gpio.output(c,gpio.HIGH)
                gpio.output(d,gpio.LOW)
                print('slight left turn')
            
            elif cx >= 387 and cx < 473: # forward
                #gpio.output(a,gpio.LOW)
                gpio.output(b,gpio.LOW)
                gpio.output(c,gpio.HIGH)
                gpio.output(d,gpio.HIGH)           
                print('forward')
                
            elif cx >= 473 and cx < 559: # slight right turn
                #gpio.output(a,gpio.LOW)
                gpio.output(b,gpio.HIGH)
                gpio.output(c,gpio.LOW)
                gpio.output(d,gpio.LOW)            
                print('slight right turn')

            elif cx >= 559 and cx < 860: # right turn
                #gpio.output(a,gpio.LOW)
                gpio.output(b,gpio.HIGH)
                gpio.output(c,gpio.LOW)
                gpio.output(d,gpio.HIGH)
                print('right turn')

            else:
                #gpio.output(a,gpio.LOW)
                gpio.output(b,gpio.LOW)
                gpio.output(c,gpio.LOW)
                gpio.output(d,gpio.LOW)
                print('0000')

        else:
            gpio.output(b,gpio.LOW)
            gpio.output(c,gpio.LOW)
            gpio.output(d,gpio.LOW)
            print ('area less than 900')
    else:
        gpio.output(b,gpio.LOW)
        gpio.output(c,gpio.LOW)
        gpio.output(d,gpio.LOW)
        print ('I don\'t see the line')


    
    cv.imshow('crop_img',crop_img)

    if cv.waitKey(1) & 0x77 == ord('q'):
        break


