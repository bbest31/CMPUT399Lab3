#################################################################################################################################
### This file implements color specific sphere tracking. It can potentially be used for tracking recovery by object detection ###
#################################################################################################################################

#Requires refactoring. This is just a "proof of concept" implementation

import cv2
import numpy as np

#####HSV Colour Ranges#################
#If the ball is red
# lower mask (0-10)
redLowerLowMask = (0,50,50)
redUpperLowMask = (10,255,255)
# high mask (170-180)
redLowerHighMask = (170,50,50)
redUpperHighMask = (180,255,255)

#If the ball is blue
blueLowMask = (100,150,0)
blueHighMask = (140,255,255)

#If the ball is orange
orangeLowMask= (5, 50, 50)
orangeHighMask= (20, 255, 255)
########################################


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
    #Uncomment for gaussian blur
    #blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    blurred = cv2.medianBlur(frame,11)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    #Red Tracking
    lowMask = cv2.inRange(hsv, redLowerLowMask, redUpperLowMask)
    highMask = cv2.inRange(hsv, redLowerHighMask, redUpperHighMask)
    #Join both of the red masks
    mask =   highMask + lowMask 
    #Uncomment for orange tracking
    #mask = cv2.inRange(hsv, orangeLowMask, orangeHighMask)
    #Uncomment for blue tracking
    #mask = cv2.inRange(hsv, blueLowMask, blueHighMask)
    #Perform erosion and dilation in the image (in 11x11 pixels squares) in order
    #To reduce the "blips" on the mask
    mask = cv2.erode(mask, np.ones((11, 11),np.uint8), iterations=2)
    mask = cv2.dilate(mask, np.ones((11, 11),np.uint8), iterations=5)
    #Mask the blurred image so that we only consider the areas with the desired colour
    masked_blurred = cv2.bitwise_and(blurred,blurred, mask= mask)
    #Convert the masked image to gray scale (Required by HoughCircles routine)
    result = cv2.cvtColor(masked_blurred, cv2.COLOR_BGR2GRAY)

    # detect circles in the image using Canny edge and Hough transform
    circles = cv2.HoughCircles(result, cv2.HOUGH_GRADIENT, 1.5, 300, param1=100, param2=20, minRadius=20, maxRadius=200)
    
    # ensure at least some circles were found
    if circles is not None:
        # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
        # loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:
            print("Circles")
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            #The circles and rectangles are drawn on the original image.
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
 
	#Shows the image we are feeding to the hough circles
    cv2.imshow("Masked And Blurred", result)
	#Shows the original image with the Median flilter applied to it
    cv2.imshow("Blurred", blurred)
    #Shows the original image with the detected circles drawn.
    cv2.imshow("Result", frame)
    #Shows the actual mask we are using
    cv2.imshow("Mask", mask)



    i+=1
    # check if esc key pressed
    key = cv2.waitKey(20)
    if key == 27:
        cv2.destroyWindow("webcam")
        break