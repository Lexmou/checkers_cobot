# coding: utf-8

import cv2
import numpy as np

def nothing(x):
    pass

def trackbars():
    video_capture = cv2.VideoCapture(0)
    cv2.namedWindow('image')

    #easy assigments
    hh='Hue High'
    hl='Hue Low'
    sh='Saturation High'
    sl='Saturation Low'
    vh='Value High'
    vl='Value Low'

    cv2.createTrackbar(hl, 'image', 0, 179, nothing)
    cv2.createTrackbar(hh, 'image', 0, 179, nothing)
    cv2.createTrackbar(sl, 'image', 0, 255, nothing)
    cv2.createTrackbar(sh, 'image', 0, 255, nothing)
    cv2.createTrackbar(vl, 'image', 0, 255, nothing)
    cv2.createTrackbar(vh, 'image', 0, 255, nothing)

    ret,frame = video_capture.read()
    while ret == True:
        ret,frame = video_capture.read()
        frame = cv2.GaussianBlur(frame, (5,5), 0)
        #convert to HSV from BGR
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


        #read trackbar positions for all
        hul=cv2.getTrackbarPos(hl, 'image')
        huh=cv2.getTrackbarPos(hh, 'image')
        sal=cv2.getTrackbarPos(sl, 'image')
        sah=cv2.getTrackbarPos(sh, 'image')
        val=cv2.getTrackbarPos(vl, 'image')
        vah=cv2.getTrackbarPos(vh, 'image')
        #make array for final values
        HSVLOW = np.array([hul, sal, val])
        HSVHIGH = np.array([huh, sah, vah])

        #apply the range on a mask
        mask = cv2.inRange(hsv, HSVLOW, HSVHIGH)
        res = cv2.bitwise_and(frame, frame, mask =mask)

        r = 700.0 / res.shape[1]
        dim = (700, int(res.shape[0] * r))
        res = cv2.resize(res, dim, interpolation = cv2.INTER_AREA)

        cv2.imshow('image', res)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
    video_capture.release()
    cv2.destroyAllWindows()
    return res