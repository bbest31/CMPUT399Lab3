#################################################################################################################################
### This file implements color specific sphere tracking. It can potentially be used for tracking recovery by object detection ###
#################################################################################################################################

#Requires refactoring. This is just a "proof of concept" implementation

import cv2
import numpy as np
#For Red
# lower mask (0-10)
redLowerLowMask = (0,50,50)
redUpperLowMask = (10,255,255)

# high mask (170-180)
redLowerHighMask = (170,50,50)
redUpperHighMask = (180,255,255)

# for blue
blueLowerLowMask = (100,150,0)
blueUpperLowMask = (140,255,255)

#Organge
oragngeLowMask= (5, 50, 50)
orangeHighMask= (20, 255, 255)

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
    #blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    blurred = cv2.medianBlur(frame,11)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    lowMask = cv2.inRange(hsv, redLowerLowMask, redUpperLowMask)
    highMask = cv2.inRange(hsv, redLowerHighMask, redUpperHighMask)
    mask =   highMask + lowMask 
    #mask = cv2.inRange(hsv, oragngeLowMask, orangeHighMask)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    mask = cv2.erode(mask, np.ones((11, 11),np.uint8), iterations=2)
    mask = cv2.dilate(mask, np.ones((11, 11),np.uint8), iterations=5)
    res = cv2.bitwise_and(blurred,blurred, mask= mask)
    res2 = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    gray_res =  cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    # detect circles in the image
    circles = cv2.HoughCircles(res2, cv2.HOUGH_GRADIENT, 1.5, 300, param1=100, param2=20, minRadius=20, maxRadius=200)
    
    # ensure at least some circles were found
    if circles is not None:
        # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
        # loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:
            print("Circles")
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
 
	# show the output image
    cv2.imshow("output", res2)
    
    cv2.imshow("webcam", frame)
    cv2.imshow("blurred", blurred)
    cv2.imshow("mask", mask)
    cv2.imshow("and", res)


    i+=1
    # check if esc key pressed
    key = cv2.waitKey(20)
    if key == 27:
        cv2.destroyWindow("webcam")
        break