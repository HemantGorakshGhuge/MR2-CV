import numpy as np
import cv2 as cv
import RPi.GPIO as gpio
import time

video_capture = cv.VideoCapture(-1)
video_capture.set(3,1080)
video_capture.set(4,480)
video_capture.set(5,90)

a=22 # colour line (red and blue)
b=23 
c=21
d=19

e=24 # 90 degree turn

l=40 # led

gpio.setwarnings(False)
gpio.setmode(gpio.BOARD)

gpio.setup(a,gpio.OUT)
gpio.setup(b,gpio.OUT)
gpio.setup(c,gpio.OUT)
gpio.setup(d,gpio.OUT)
gpio.setup(e,gpio.OUT)

gpio.setup(l, gpio.OUT)

gpio.output(a,gpio.LOW)
gpio.output(b,gpio.LOW)
gpio.output(c,gpio.LOW)
gpio.output(d,gpio.LOW)

gpio.output(e,gpio.LOW)

gpio.output(l,gpio.HIGH)
ctrl = 0
flag = 0

def line_follow():
    
    if cx >= 0 and cx < 301: # left turn               
        flag = 1
        
    elif cx >= 301 and cx < 387: # slight left turn        
        flag = 2
            
    elif cx >= 387 and cx < 473: # forward       
        flag = 3
                       
    elif cx >= 473 and cx < 559: # slight right turn        
        flag = 4

    elif cx >= 559 and cx < 860: # right turn
        flag = 5

    print('IN LINE FOLLOW FUNCTION')
    return(flag)

while(True):

    start = time.time()        
    
    ret, frame = video_capture.read()

    crop_img = frame[0:480, 0:1080]

    hsv = cv.cvtColor(crop_img, cv.COLOR_BGR2HSV)
    
    lower_white = np.array([100, 0, 150])
    upper_white = np.array([118, 100, 255])

    #lower_blue = np.array([100, 150, 50])
    #upper_blue = np.array([118, 255, 255])

    mask_white = cv.inRange(hsv, lower_white, upper_white)
    #cv.imshow('mask white', mask_white)
    
    kernel = np.ones((10,10), np.uint8)
    white_line = cv.morphologyEx(mask_white, cv.MORPH_OPEN, kernel)
    #cv.imshow('white line', white_line)

    _, contours_white, _= cv.findContours(white_line.copy(), 1, cv.CHAIN_APPROX_NONE)
    #print ("length of contours_white = " + str(len(contours_white)))

    lower_red = np.array([160, 50, 50])
    upper_red = np.array([180, 255, 255])
    
    mask_red = cv.inRange(hsv, lower_red, upper_red)
    #cv.imshow('mask red', mask_red)

    kernel = np.ones((3,3), np.uint8)
    red_line = cv.morphologyEx(mask_red, cv.MORPH_OPEN, kernel)
    #cv.imshow('red line', red_line)

    _, contours_red, _= cv.findContours(red_line.copy(), 1, cv.CHAIN_APPROX_NONE)
    #print ("length of contours_red = " + str(len(contours_red))


    
    if len(contours_white) > 0:
        
        con_white = max(contours_white, key = cv.contourArea)
        area_white = cv.contourArea(con_white)
        print('area_white = ' + str(area_white))

        if (area_white > 8000):
            blackbox = cv.minAreaRect(con_white)
            (x_min, y_min), (w_min, h_min), ang = blackbox

            if ang < -45 :
                ang = 90 + ang
          
            if w_min < h_min and ang > 0:    
                ang = (90-ang)*-1
                      
            if w_min > h_min and ang < 0:
                ang = 90 + ang
            
            ang = int(ang)
            print('ang = '+ str(ang))
            cv.putText(crop_img, str(ang),(50,50), cv.FONT_HERSHEY_SIMPLEX, 2, (20, 20, 250), 4)

            if (ang > 75 and ang < 100) or (ang < -75 and ang > -100):
                gpio.output(e,gpio.HIGH)
                gpio.output(l,gpio.LOW)
                ctrl+=1
                print('90 turn signal angle detection')
            
            M_white = cv.moments(con_white)
            
            if M_white['m00'] == 0:
                M_white['m00'] == 0.0000001;

            cx_white = int(M_white['m10']/M_white['m00'])
            cy_white = int(M_white['m01']/M_white['m00'])

            cv.line(crop_img, (cx_white,0), (cx_white,480), (255,0,0),3)
            cv.line(crop_img, (0,cy_white), (1080,cy_white), (255,0,0),3)

            cv.drawContours(crop_img, contours_white, -1, (0,255,0), 3)

            print ("cx_white = " +str(cx_white))

            cx = cx_white
            
            flag = line_follow()
            
        else:
            print ('area less than 8000')
    else:
        print ('length of white contours < 0')


    if len(contours_red) > 0:

        con_red = max(contours_red, key = cv.contourArea)
        area_red = cv.contourArea(con_red)
        print('area_red = ' + str(area_red))

        if (area_red > 15000):
            M_red = cv.moments(con_red)

            if M_red['m00'] == 0:
                M_red['m00'] == 0.0000001;

            cx_red = int(M_red['m10']/M_red['m00'])
            cy_red = int(M_red['m01']/M_red['m00'])

            cv.line(crop_img, (cx_red,0), (cx_red,480), (255,255,0),3)
            cv.line(crop_img, (0,cy_red), (1080,cy_red), (255,255,0),3)

            cv.drawContours(crop_img, contours_red, -1, (0,255,255), 3)
        
            gpio.output(a, gpio.HIGH)
            print('red detected area more than 15000')

        else:
            gpio.output(a, gpio.LOW)
            print('area less than 15000 (red)')
    else:
        gpio.output(a,gpio.LOW)
        print('length of red contours < 0')

    cv.imshow('crop_img', crop_img)
    
    if flag == 1:
        gpio.output(b,gpio.LOW)
        gpio.output(c,gpio.LOW)
        gpio.output(d,gpio.HIGH)
        print('left turn')
        
    if flag == 2:
        gpio.output(b,gpio.LOW)
        gpio.output(c,gpio.HIGH)
        gpio.output(d,gpio.LOW)
        print('slight left turn')
    
    if flag == 3:
        gpio.output(b,gpio.LOW)
        gpio.output(c,gpio.HIGH)
        gpio.output(d,gpio.HIGH)           
        print('forward')
        
    if flag == 4:
        gpio.output(b,gpio.HIGH)
        gpio.output(c,gpio.LOW)
        gpio.output(d,gpio.LOW)            
        print('slight right turn')
        
    if flag == 5:
        gpio.output(b,gpio.HIGH)
        gpio.output(c,gpio.LOW)
        gpio.output(d,gpio.HIGH)
        print('right turn')
        
    print('ctrl = ' + str(ctrl))
    print('flag = ' + str(flag))
    end = time.time()

    print("time execution " + str(end-start))
    
    if cv.waitKey(1) & 0x77 == ord('q'):
        break



