import numpy as np
import cv2 as cv

video_capture = cv.VideoCapture(-1)
video_capture.set(3,640)
video_capture.set(4,480)

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

        print ("cx = " + str(cx))


    else:
        print ('I don\'t see the white colour')

    
    cv.imshow('crop_img',crop_img)

    if cv.waitKey(1) & 0x77 == ord('q'):
        break

