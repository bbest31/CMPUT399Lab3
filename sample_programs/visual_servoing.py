import cv2
import sys
 
(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

def choose_tracking_method(index,minor_ver):
    tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
    tracker_type = tracker_types[index]

    if int(minor_ver) < 3:
        tracker = cv2.Tracker_create(tracker_type)
    else:
        if tracker_type == 'BOOSTING':
            tracker = cv2.TrackerBoosting_create()
        if tracker_type == 'MIL':
            tracker = cv2.TrackerMIL_create()
        if tracker_type == 'KCF':
            tracker = cv2.TrackerKCF_create()
        if tracker_type == 'TLD':
            tracker = cv2.TrackerTLD_create()
        if tracker_type == 'MEDIANFLOW':
            tracker = cv2.TrackerMedianFlow_create()
        if tracker_type == 'GOTURN':
            tracker = cv2.TrackerGOTURN_create()
        if tracker_type == 'MOSSE':
            tracker = cv2.TrackerMOSSE_create()
        if tracker_type == "CSRT":
            tracker = cv2.TrackerCSRT_create()
    return tracker, tracker_type

if __name__ == '__main__' :
 
    # Set up tracker.
    tracker, tracker_type = choose_tracking_method(2,minor_ver)
    
    cv2.namedWindow("webcam")
    vc = cv2.VideoCapture(1)


    if vc.isOpened(): # try to get the first frame
        rval, frame = vc.read()
    else:
        rval = False

    i = 0
    while rval and i < 100:
        # Read a new frame
        rval, frame = vc.read()
        cv2.imshow("webcam", frame)
        i = i + 1

    target_bbox = cv2.selectROI("webcam", frame, False)
    target_point =(int(target_bbox[0] + target_bbox[2]/2),int(target_bbox[1] + target_bbox[3]/2)) 

    bbox = cv2.selectROI("webcam", frame, False)

    # Initialize tracker with first frame and bounding box
    rval = tracker.init(frame, bbox)
 
    while rval:
        # Read a new frame
        rval, frame = vc.read()
        # Start timer
        timer = cv2.getTickCount()
        # Update tracker
        ok, bbox = tracker.update(frame)
        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
        # Draw bounding box
        if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            feature_point = (int(bbox[0] + bbox[2]/2),int(bbox[1] + bbox[3]/2)) 
            cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
            cv2.rectangle(frame, (int(bbox[0] + bbox[2]/2) - 2, int(bbox[1] + bbox[3]/2) - 2), (int(bbox[0] + bbox[2]/2) + 2,  int(bbox[1] + bbox[3]/2) + 2), (0, 128, 255), -1) 
            cv2.putText(frame, "Feature Point (x,y): "  + str(feature_point), (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            cv2.putText(frame, "Target Point (x,y): "  + str(target_point), (100,110), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            cv2.rectangle(frame, (int(target_bbox[0] + target_bbox[2]/2) - 2, int(target_bbox[1] + target_bbox[3]/2) - 2), (int(target_bbox[0] + target_bbox[2]/2) + 2,  int(target_bbox[1] + target_bbox[3]/2) + 2), (0, 128, 255), -1) 

        else :
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
        # Display tracker type on frame
        cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
        # Display FPS on frame
        cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
        # Display result
        cv2.imshow("webcam", frame)
        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break