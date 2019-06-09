import numpy as np
import cv2 as cv

video_capture = cv.VideoCapture(-1)
video_capture.set(3,1080)
video_capture.set(4,480)
video_capture.set(5,90)

lower_white = np.array([100, 0, 200])
upper_white = np.array([118, 100, 255]) # S = white and blue V = white and yellow
shape_white = (10,10)

def find_contours(lower, upper, shape):

    mask = cv.inRange(hsv, lower, upper)

    kernel = np.ones(shape, np.uint8)
    line = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

    _, contours, _= cv.findContours(line.copy(), 1, cv.CHAIN_APPROX_NONE)
    
    return(contours)

while(True):

    ret, frame = video_capture.read()
    
    crop_img = frame[0:480, 0:1080]

    hsv = cv.cvtColor(crop_img, cv.COLOR_BGR2HSV)
    
    contours_white = find_contours(lower_white, upper_white, shape_white)
    
    if len(contours_white) > 0:
        
        con_white = max(contours_white, key = cv.contourArea)
        area_white = cv.contourArea(con_white)
        print('area_white = ' + str(area_white))

        M_white = cv.moments(con_white)
            
        if M_white['m00'] == 0:
            M_white['m00'] == 0.0000001;

        cx_white = int(M_white['m10']/M_white['m00'])
        cy_white = int(M_white['m01']/M_white['m00'])

        cv.line(crop_img, (cx_white,0), (cx_white,480), (255,0,0),3)
        cv.line(crop_img, (0,cy_white), (1080,cy_white), (255,0,0),3)

        cv.drawContours(crop_img, contours_white, -1, (0,255,0), 3)

        print ("cx_white = " +str(cx_white))
                        
    else:
        print ('length of white contours < 0')

    
    cv.imshow('crop_img',crop_img)

    if cv.waitKey(1) & 0x77 == ord('q'):
        break

