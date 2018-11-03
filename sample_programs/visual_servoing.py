import cv2
import sys
import numpy as np
from server import Server
from rectangle import Rectangle
from threading import Event, Thread
from time import sleep

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

server = Server("127.0.0.1",9999)

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

#Feature point and target point are both (u,v) tuples
#Returns an error tuple (error_in_u, error_in_v)
def compute_error(current_position, target_position):
    return (target_position[0] - current_position[0], target_position[1] - current_position[1])

#This is an implementation of what we have in http://ugweb.cs.ualberta.ca/~vis/courses/robotics/lectures/lec10VisServ.pdf (pages 25 and 26)
def broyden_update(jacobian_matrix, position_vector, angle_vector, alpha):
    #We need to make sure these are numpy objects.
    position_vector = np.array(position_vector)
    angle_vector = np.array(angle_vector)
    jacobian_matrix = np.mat(jacobian_matrix)
    #Compute the fraction term separately (for clarity)
    fraction = ((position_vector - (np.array(jacobian_matrix) * angle_vector))*angle_vector)/(angle_vector*angle_vector)
    #Perform the rank 1 update. This will return an updated jacobian matrix as a numpy matrix
    return  jacobian_matrix + np.mat(alpha * fraction)

if __name__ == '__main__' :
 
    # Set up tracker.
    tracker, tracker_type = choose_tracking_method(2,minor_ver)
    
    vc = cv2.VideoCapture(0)

    if vc.isOpened(): # try to get the first frame
        rval, frame = vc.read()
    else:
        rval = False

    i = 0
    while rval and i < 100:
        # Read a new frame
        rval, frame = vc.read()
        i = i + 1

    cv2.namedWindow("webcam")


    #In here the user draws a bounding box around the end effector. We can consider using our shape tracking too
    cv2.putText(frame, "Select End Effector", (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
    #bbox is an array represending a rectangle: [x, y, height, width]
    bbox = cv2.selectROI("webcam",frame, False)
    bounding_rectangle = Rectangle(bbox)

    # Initialize tracker with first frame and bounding box
    rval = tracker.init(frame, bounding_rectangle.array_representation)

    ########Estimate Initial Jacobian##############

    #This is the initial position. It is the centre of bounding box around the end effector stored as an (u,v) tuple
    feature_point = bounding_rectangle.centre 

    #These are angles that we will use to estimate the initial image jacobian.
    #They can remain hardcoded as they will only be used once. 
    base_angle = 5
    joint_angle = 10

    ############This will compute the first column of the initial jacobian. Will encapsulate in a function.#################

    #Moves the base by the desired angle, while the joint is fixed. This will help us estimate the first column of the Jacobian.
    #The movement command is sent to the client (EV3 Brick) via socket. This will block until the EV3 reports back to us that the
    #movement has been completed or a timout occurs.
    robot_movement_thread = Thread(target=server.sendData, args=(base_angle,0))
    robot_movement_thread.start()
    #previous_feature_point is a (u,v) tuple
    previous_feature_point = feature_point
    #While the thread handling the robot movement is alive we assume the robot is moving.
    #We update the trackers and the frames
    while robot_movement_thread.is_alive() and rval:
        rval, frame = vc.read()
        ok, bbox = tracker.update(frame)
        bounding_rectangle = Rectangle(bbox)
        if ok:
            # Tracking success
            #Update the current position of the end effector
            feature_point = bounding_rectangle.centre 
            #Draw rectangles
            cv2.rectangle(frame, bounding_rectangle.top_left, bounding_rectangle.bottom_right, (255,0,0), 2, 1)
            cv2.rectangle(frame, (int(feature_point[0]) - 2, int(feature_point[1]) - 2), (int(feature_point[0]) + 2,  int(feature_point[1]) + 2), (0, 128, 255), -1) 
        else :
            # Tracking failure
            print("Tracking Failure")
            #Recover by using detection? Or else we have to terminate the program here.
        cv2.imshow("webcam", frame)
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break

    #Compute delta of u and delta v. Both points are stored as (u,v) tuples
    #Then divide each delta by the angle by which we just rotated. This will give us the first column of the initial Jacobian
    delta_u = feature_point[0] - previous_feature_point[0]
    delta_v = feature_point[1] - previous_feature_point[1]
    jacobian_column_1 = [delta_u / base_angle , delta_v / base_angle]
    print("Column1: " + str(jacobian_column_1))

    #########This will compute the second column of the initial Jacobian. Will encapsulate in a function.###############
 
    #Moves the base by the desired angle, while the joint is fixed. This will help us estimate the first column of the Jacobian.
    #The movement command is sent to the client (EV3 Brick) via socket. This will block until the EV3 reports back to us that the
    #movement has been completed or a timout occurs.
    robot_movement_thread = Thread(target=server.sendData, args=(0,joint_angle))
    robot_movement_thread.start()
    #This is also a (u,v) tuple. Keeps current position to be used in the delta later
    previous_feature_point = feature_point
    #While the thread handling the robot movement is alive we assume the robot is moving.
    #We update the trackers and the frames   
    while robot_movement_thread.is_alive() and rval:
        rval, frame = vc.read()
        ok, bbox = tracker.update(frame)
        bounding_rectangle = Rectangle(bbox)
        if ok:
            # Tracking success
            #Update the current position of the end effector
            feature_point = bounding_rectangle.centre 
            #Draw rectangles
            cv2.rectangle(frame, bounding_rectangle.top_left, bounding_rectangle.bottom_right, (255,0,0), 2, 1)
            cv2.rectangle(frame, (int(feature_point[0]) - 2, int(feature_point[1]) - 2), (int(feature_point[0]) + 2,  int(feature_point[1]) + 2), (0, 128, 255), -1) 
        else :
            # Tracking failure
            print("Tracking Failure")
            #Recover by using detection? Or else we have to terminate the program here.
        cv2.imshow("webcam", frame)
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break       
    #Compute delta of u and delta v. Both points are stored as (u,v) tuples
    #Then divide each delta by the angle by which we just rotated. This will give us the first column of the initial Jacobian
    delta_u = feature_point[0] - previous_feature_point[0]
    delta_v = feature_point[1] - previous_feature_point[1]
    jacobian_column_2 = [delta_u / joint_angle , delta_v / joint_angle]
    print("Column2: " + str(jacobian_column_2))
    #Join the two columns and transpose them (they were stored as row vectors)
    jacobian_matrix = np.mat((jacobian_column_1, jacobian_column_2)).getT()
    print(jacobian_matrix)
    #############End of Initial Jacobian Estimation#########################
    
    cv2.putText(frame, "Select Target Point", (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
    target_bbox = cv2.selectROI("webcam", frame, False)
    target_bounding_rectange = Rectangle(target_bbox)
    target_point = target_bounding_rectange.centre

    #############Start Visual Servoing##########################

    #Initial Error
    error_vector = compute_error(feature_point, target_point)
    #Constants for scaling the results (as shown in the last lab)
    alpha = 5
    scaling = 0.8
    
    #This loop mimics the process outlined in http://ugweb.cs.ualberta.ca/~vis/courses/robotics/lectures/lec10VisServ.pdf page 26
    while True:
        #Solve the linear system e = J*q -> q = scaling * (inverse(J)*e). Where scaling is just a scaling parameter we adjust empirically, in order
        #to have some degree of control over how large are the angles.
        #angles is a vector of the form [base_angle, joint_angle]
        angles = scaling * np.linalg.solve(jacobian_matrix,error_vector)
        print(angles)
        #Send a move command with the joint_angles vector to the robot
        robot_movement_thread = Thread(target=server.sendData, args=(angles[0], angles[1]))
        robot_movement_thread.start()
        #While robot is moving
        #We store the current position of the end effector to calculate the delta
        previous_feature_point = feature_point
        while robot_movement_thread.is_alive() and rval:
            # Read a new frame
            rval, frame = vc.read()
            # Start timer
            timer = cv2.getTickCount()
            # Update tracker
            ok, bbox = tracker.update(frame)
            bounding_rectangle = Rectangle(bbox)
            # Calculate Frames per second (FPS)
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
            if ok:
                # Tracking success
                #Update end effector position
                feature_point = bounding_rectangle.centre 
                # Draw bounding box
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
        
        #Broyden Update
        position_delta = (feature_point[0] - previous_feature_point[0], feature_point[1] - previous_feature_point[1]))
        print("Prev Feature" + str(previous_feature_point))
        print("Current POs" + str(feature_point)) 
        print("Delta: " + str(position_delta))
        #Perform a rank 1 update of the jacobian
        jacobian_matrix = broyden_update(jacobian_matrix, position_delta , angles, alpha)

        #Compute the error between the current position of the end effector and the target
        error_vector = compute_error(feature_point, target_point)
        print("error vector: " + str(error_vector))



