from collections import deque

import numpy as np


import cv2

#For Red
# lower mask (0-10)
redLowerLowMask = (0,50,50)
redUpperLowMask = (10,255,255)

# high mask (170-180)
redLowerHighMask = (170,50,50)
redUpperHighMask = (180,255,255)

# for blue
blueLowerLowMask = (110,50,50)
blueUpperLowMask = (130,255,255)

cv2.namedWindow("webcam")
vc = cv2.VideoCapture(0)


if vc.isOpened(): # try to get the first frame
    rval, frame = vc.read()
else:
    rval = False
i = 0
while rval:
    # handle current frame

    rval, frame = vc.read()

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    lowMask = cv2.inRange(hsv, redLowerLowMask, redUpperLowMask)
    highMask = cv2.inRange(hsv, redLowerHighMask, redUpperHighMask)
    mask =   highMask + lowMask 

    mask = cv2.erode(mask, np.ones((11, 11),np.uint8), iterations=2)
    mask = cv2.dilate(mask, np.ones((11, 11),np.uint8), iterations=2)


    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    center = None



    if len(cnts) > 0:

        c = max(cnts, key=cv2.contourArea)

        ((x, y), radius) = cv2.minEnclosingCircle(c)



        if radius > 10:

            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)



    cv2.imshow("Frame", frame)

    cv2.imshow("Mask", mask)



    key = cv2.waitKey(1) & 0xFF



    if key == ord("q"):

        break



camera.release()

cv2.destroyAllWindows()