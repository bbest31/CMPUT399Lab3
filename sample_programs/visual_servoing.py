import cv2
import sys
from server import Server
from rectangle import Rectangle

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

server = Server()

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
        

def initial_jacobian_column(base_angle, joint_angle):
    print ("Function to estimate initial jacobian column")

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
    while rval and i < 50:
        # Read a new frame
        rval, frame = vc.read()
        cv2.imshow("webcam", frame)
        i = i + 1

    #In here the user draws a bounding box around the end effector. We can consider using our shape tracking too
    cv2.putText(frame, "Select End Effector", (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
    #bbox is an array represending a rectangle: [x, y, height, width]
    bbox = cv2.selectROI("webcam", frame, False, True)
    bounding_rectangle = Rectangle(bbox)
    #Should we draw the bounding box?

    ########Estimate Initial Jacobian##############

    #This is the initial position. It is the centre of bounding box around the end effector stored as an (u,v) tuple
    feature_point = bounding_rectangle.centre 
    #These are angles that we will use to estimate the initial image jacobian.
    #They can remain hardcoded as they will only be used once. 
    base_angle = 6
    joint_angle = 9

    ############This will compute the first column. Will encapsulate in a function.#################

    #Moves the base by the desired angle, while the joint is fixed. This will help us estimate the first column of the Jacobian.
    #The movement command is sent to the client (EV3 Brick) via socket. This will block until the EV3 reports back to us that the
    #movement has been completed or a timout occurs.
    server.sendData(base_angle,0)
    #After the move is complete, we read the data from the camera to determine delta of u and v for the first column.
    rval, frame = vc.read()
    #This is also a (u,v) tuple
    previous_feature_point = feature_point
    if rval:
        cv2.imshow("webcam", frame)
        ok, bbox = tracker.update(frame)
        bounding_rectangle = Rectangle(bbox)
        if ok:
            # Tracking success
            feature_point = bounding_rectangle.centre 
            #Should we show updated bounding box on frame?
        else :
            # Tracking failure
            print("Tracking Failure")
            #Recover by using detection? Or else we have to terminate the program here.
    #Compute delta of u and delta v. Both points are stored as (u,v) tuples
    #Then divide each delta by the angle by which we just rotated. This will give us the first column of the initial Jacobian
    delta_u = feature_point[0] - previous_feature_point[0]
    delta_v = feature_point[1] - previous_feature_point[1]
    jacobian_column_1 = [delta_u / base_angle , delta_v / base_angle]

    #########This will compute the second column. Will encapsulate in a function.###############
 
    #Moves the base by the desired angle, while the joint is fixed. This will help us estimate the first column of the Jacobian.
    #The movement command is sent to the client (EV3 Brick) via socket. This will block until the EV3 reports back to us that the
    #movement has been completed or a timout occurs.
    server.sendData(0,joint_angle)
    #After the move is complete, we read the data from the camera to determine delta of u and v for the first column.
    rval, frame = vc.read()
    #This is also a (u,v) tuple
    previous_feature_point = feature_point
    if rval:
        cv2.imshow("webcam", frame)
        ok, bbox = tracker.update(frame)
        bounding_rectangle = Rectangle(bbox)
         if ok:
            # Tracking success
            feature_point = bounding_rectangle.centre 
            #Should we show updated bounding box on frame?
        else :
            # Tracking failure
            print("Tracking Failure")
            #Recover by using detection? Or else we have to terminate the program here.
    #Compute delta of u and delta v. Both points are stored as (u,v) tuples
    #Then divide each delta by the angle by which we just rotated. This will give us the first column of the initial Jacobian
    delta_u = feature_point[0] - previous_feature_point[0]
    delta_v = feature_point[1] - previous_feature_point[1]
    jacobian_column_2 = [delta_u / joint_angle , delta_v / joint_angle]

    jacobian_matrix = np.asmatrix(np.column_stack((jacobian_column_1, jacobian_column_2)))

    #############End of Initial Jacobian Estimation#########################
    
    cv2.putText(frame, "Select Target Point", (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
    target_bbox = cv2.selectROI("webcam", frame, False, True)
    target_bounding_rectange = Rectangle(target_bbox)
    target_point = target_bounding_rectange.centre



    # Initialize tracker with first frame and bounding box
    rval = tracker.init(frame, bbox)
 
    while rval:
        # Read a new frame
        rval, frame = vc.read()
        # Start timer
        timer = cv2.getTickCount()
        # Update tracker
        ok, bbox = tracker.update(frame)
        bounding_rectangle = Rectangle(bbox)
        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
        # Draw bounding box
        if ok:
            # Tracking success
            feature_point = bounding_rectangle.centre 
            cv2.rectangle(frame, bounding_rectangle.top_left, bounding_rectangle.bottom_right, (255,0,0), 2, 1)
            cv2.rectangle(frame, (int(feature_point[0]) - 2, int(feature_point[1]) - 2), (int(feature_point[0]) + 2,  int(feature_point[1]) + 2), (0, 128, 255), -1) 
            cv2.putText(frame, "Feature Point (x,y): "  + str(feature_point), (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            cv2.putText(frame, "Target Point (x,y): "  + str(target_point), (100,110), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            cv2.rectangle(frame, (int(target_point[0]) - 2, int(target_point[1]) - 2), (int(target_point[0]) + 2,  int(target_point[1]) + 2), (0, 128, 255), -1) 

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