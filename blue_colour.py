import numpy as np
import cv2 as cv

video_capture = cv.VideoCapture(-1)
video_capture.set(3,640)
video_capture.set(4,480)

while(True):

    ret, frame = video_capture.read()

    crop_img = frame[0:480, 0:1080]

    hsv = cv.cvtColor(crop_img, cv.COLOR_BGR2HSV)

    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([118, 255, 255])

    mask_blue = cv.inRange(hsv, lower_blue, upper_blue)
    #cv.imshow('mask blue', mask_blue)

    kernel = np.ones((3,3), np.uint8)
    blue_line = cv.morphologyEx(mask_blue, cv.MORPH_OPEN, kernel)
    #cv.imshow('blue line', blue_line)

    _, contours_blue, _= cv.findContours(blue_line.copy(), 1, cv.CHAIN_APPROX_NONE)
    #print ("length of contours_blue = " + str(len(contours_blue)))


    if len(contours_blue) > 0:
        
        con_blue = max(contours_blue, key = cv.contourArea)
        area_blue = cv.contourArea(con_blue)
        print('area_blue = ' + str(area_blue))

        if (area_blue > 15000):
            M_blue = cv.moments(con_blue)

            if M_blue['m00'] == 0:
                M_blue['m00'] == 0.0000001;

            cx_blue = int(M_blue['m10']/M_blue['m00'])
            cy_blue = int(M_blue['m01']/M_blue['m00'])

            cv.line(crop_img, (cx_blue,0), (cx_blue,480), (0,0,200),3)
            cv.line(crop_img, (0,cy_blue), (1080,cy_blue), (0,0,200),3)

            cv.drawContours(crop_img, contours_blue, -1, (0,255,255), 3)
            
            gpio.output(a, gpio.HIGH)
            print('blue detected area more than 15000')

        else:
            gpio.output(a, gpio.LOW)
            print('area less than 15000(blue)')

    else:
        gpio.output(a, gpio.LOW)
        print('length of blue contours < 0')


    if cv.waitKey(1) & 0x77 == ord('q'):
        break


