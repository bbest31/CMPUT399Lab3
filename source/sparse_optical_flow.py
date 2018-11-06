import numpy as np
import cv2
from rectangle import Rectangle


cv2.namedWindow("webcam")


cap = cv2.VideoCapture(0)
# params for ShiTomasi corner detection
feature_params = dict( maxCorners = 100,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )

# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15,15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Create some random colors
color = np.random.randint(0,255,(100,3))

# Take first few frames
rval, frame = cap.read()
i = 0
while rval and i < 100:
    # Read a new frame
    rval, frame = cap.read()
    i = i + 1



bbox = cv2.selectROI("webcam",frame, False)
bounding_rectangle = Rectangle(bbox)

initial_mask = np.zeros_like(frame)     
# Crop image
channel_count = frame.shape[2]  
ignore_mask_color = (255,)*channel_count
corners = np.array([[bounding_rectangle.top_left, bounding_rectangle.top_right, bounding_rectangle.bottom_left, bounding_rectangle.bottom_right]],dtype=np.int32)

cv2.fillPoly(initial_mask, corners, ignore_mask_color)
#cropped_frame = frame[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
masked_image = cv2.bitwise_and(frame, initial_mask)
gray_masked_image = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
p0 = cv2.goodFeaturesToTrack(gray_masked_image, mask = None, **feature_params)

# Create a mask image for drawing purposes
mask = np.zeros_like(frame)
old_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

while(1):
    ret,frame = cap.read()
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # calculate optical flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

    # Select good points
    good_new = p1[st==1]
    good_old = p0[st==1]

    # draw the tracks
    for i,(new,old) in enumerate(zip(good_new,good_old)):
        a,b = new.ravel()
        c,d = old.ravel()
        mask = cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
        frame = cv2.circle(frame,(a,b),5,color[i].tolist(),-1)
    img = cv2.add(frame,mask)

    cv2.imshow('webcam',img)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break

    # Now update the previous frame and previous points
    old_gray = frame_gray.copy()
    p0 = good_new.reshape(-1,1,2)

cv2.destroyAllWindows()
cap.release()