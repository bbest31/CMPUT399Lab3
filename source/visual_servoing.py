import cv2
import sys
import numpy as np
from server import Server
from rectangle import Rectangle
from threading import Event, Thread
from time import sleep
from queue import Queue

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

#Computes the delta of two vectors, where each vector is a tuple in image space (u,v)
def compute_delta(initial_vector, final_vector):
    return (final_vector[0] - initial_vector[0], final_vector[1] - initial_vector[1])

def obstacle_detected(queue):
    obstacle_detected = False
    #This loop will check all the messages in the queue
    #and return True if any of them indicate that the
    #an obstacle was detected. It will also clear the queue
    while (not queue.empty()):
        message = queue.get()
        if (message == "RESET"):
            obstacle_detected = True
    return obstacle_detected


#Returns a Rectangle Object, representing the tracker bounding box.
def move_and_track(tracker, vc, base_angle, joint_angle, target_point, server, queue):
    robot_movement_thread = Thread(target=server.sendAngles, args=(base_angle,joint_angle, queue))
    robot_movement_thread.start() 
    tracking_failed = False
    while robot_movement_thread.is_alive():
        rval, frame = vc.read()
        ok, bbox = tracker.update(frame)
        bounding_rectangle = Rectangle(bbox)
        if ok:
            # Tracking success
            #Update the current position of the end effector
            current_position = bounding_rectangle.centre 
            #Draw rectangles
            cv2.rectangle(frame, bounding_rectangle.top_left, bounding_rectangle.bottom_right, (255,0,0), 2, 1)
            cv2.rectangle(frame, (int(current_position[0]) - 2, int(current_position[1]) - 2), (int(current_position[0]) + 2,  int(current_position[1]) + 2), (0, 0, 0), -1) 
            cv2.putText(frame, "Feature Point (x,y): "  + str(feature_point), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            cv2.putText(frame, "Target Point (x,y): "  + str(target_point), (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            cv2.rectangle(frame, (int(target_point[0]) - 2, int(target_point[1]) - 2), (int(target_point[0]) + 2,  int(target_point[1]) + 2), (0, 128, 255), -1) 
            tracking_failed = False
        else :
            # Tracking failure
            print("Tracking Failure")
            tracking_failed = True

        cv2.imshow("webcam", frame)
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break
    #If we lost tracking and couldn't recover, we need the user to manually select the end effector:
    if tracking_failed:
        rval, frame = vc.read()
        cv2.putText(frame, "Tracking failure detected.", (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
        cv2.putText(frame, "Please select End Effector", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
        cv2.rectangle(frame, (int(target_point[0]) - 2, int(target_point[1]) - 2), (int(target_point[0]) + 2,  int(target_point[1]) + 2), (0, 128, 255), -1) 
        bbox = cv2.selectROI("webcam",frame, False)
        bounding_rectangle = Rectangle(bbox)

    return tracking_failed, bounding_rectangle


#This is an implementation of what we have in http://ugweb.cs.ualberta.ca/~vis/courses/robotics/lectures/lec10VisServ.pdf (pages 25 and 26)
def broyden_update(jacobian_matrix, position_vector, angle_vector, alpha):
    #We need to make sure these are numpy objects.
    position_vector = np.array(position_vector)
    angle_vector = np.array(angle_vector)
    jacobian_matrix = np.mat(jacobian_matrix)
    #Compute the fraction term separately (for clarity)
    numerator = np.outer(position_vector - jacobian_matrix.dot(angle_vector),angle_vector)
    denominator = angle_vector.dot(angle_vector)
    #This will essentially divide the 2x2 matrix in the numerator
    #By the squared norm of the the angle_vector
    fraction = numerator/denominator
    #Perform the rank 1 update. This will return an updated jacobian matrix as a numpy matrix
    return  jacobian_matrix + (alpha * fraction)

#Returns initial jacobian and postion of the end effector after the computation
#of the initial Jacobian
def initial_jacobian(tracker, vc, previous_position, target_point, server, queue):
    base_angle = 20
    joint_angle = 20
    #Calculate first column of the initial Jacobian
    #This is done by moving the base and fixing the joint
    tracking_failed, end_effector_bounding_box = move_and_track(tracker, vc, base_angle, 0, target_point, server, queue)
    current_position = end_effector_bounding_box.centre
    position_delta = compute_delta(previous_position, current_position)
    jacobian_column_1 = [position_delta[0] / base_angle, position_delta[1] / base_angle]
    #Update the previous position to be the current position of the end effector.
    previous_position = current_position
    #Calculate second column of the initial Jacobian
    #This is done by moving the joint and fixing the base
    tracking_failed, end_effector_bounding_box = move_and_track(tracker, vc, 0, joint_angle, target_point, server, queue)
    current_position = end_effector_bounding_box.centre
    position_delta = compute_delta(previous_position, current_position)
    jacobian_column_2 = [position_delta[0] / joint_angle, position_delta[1] / joint_angle]
    #Once the two columns are computed:
    #Join the two columns and transpose them (they were stored as row vectors)
    jacobian_matrix = np.mat((jacobian_column_1, jacobian_column_2)).getT()
    return jacobian_matrix, current_position


if __name__ == '__main__' :

    (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')

    server = Server(9999)
    queue = Queue()
    # Set the Safety mode HERE!#########
    safe_mode_active = True
    ####################################

    #Enable or disable safety mode on the client (Brick)
    if (safe_mode_active):
        server.sendEnableSafetyMode()
    else:
        server.sendDisableSafetyMode()

    # Set up tracker.
    tracker, tracker_type = choose_tracking_method(2,minor_ver)
    
    vc = cv2.VideoCapture(1)

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
    cv2.putText(frame, "Select End Effector", (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 127, 0), 2)
    #bbox is an array represending a rectangle: [x, y, height, width]
    bbox = cv2.selectROI("webcam",frame, False)
    bounding_rectangle = Rectangle(bbox)
    #This is the initial position. It is the centre of bounding box around the end effector stored as an (u,v) tuple
    feature_point = bounding_rectangle.centre 
    #Display feature point position and coordinates in the next frame
    rval, frame = vc.read()
    cv2.rectangle(frame, bounding_rectangle.top_left, bounding_rectangle.bottom_right, (255,0,0), 2, 1)
    cv2.rectangle(frame, (int(feature_point[0]) - 2, int(feature_point[1]) - 2), (int(feature_point[0]) + 2,  int(feature_point[1]) + 2), (0, 0, 0), -1) 
    cv2.putText(frame, "Feature Point (x,y): "  + str(feature_point), (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
    #Display prompt to select target point
    cv2.putText(frame, "Select Target Point", (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 127, 0), 2)
    target_bbox = cv2.selectROI("webcam", frame, False)
    #Store Target point the corresponding variables. These variables
    #are treated as constants
    target_bounding_rectange = Rectangle(target_bbox)
    target_point = target_bounding_rectange.centre


    # Initialize tracker with first frame and bounding box
    rval = tracker.init(frame, bounding_rectangle.array_representation)

    ###############Estimate Initial Jacobian################################

    jacobian_matrix, feature_point = initial_jacobian(tracker, vc, feature_point, target_point, server, queue)
    #If an obstacle was detected during the computation 
    if (safe_mode_active and obstacle_detected(queue)):
        print("Obstacle During Initial Jacobian!")
        jacobian_matrix, feature_point = initial_jacobian(tracker, vc, feature_point, target_point, server, queue)

    #############End of Initial Jacobian Estimation#########################
    

    #############Start Visual Servoing##########################
    #With the target point set, and the initial jacobian calculated, we can 
    #now start the actual process for visual servoing.

    #Initial Error
    error_vector = compute_delta(feature_point, target_point)
    #Constants for scaling the results (as shown in the last lab)
    alpha = 0.5
    scaling = 0.25

    #This loop mimics the process outlined in http://ugweb.cs.ualberta.ca/~vis/courses/robotics/lectures/lec10VisServ.pdf page 26
    while np.linalg.norm(error_vector) > 10:
        #Solve the linear system e = J*q -> q = scaling * (inverse(J)*e). Where scaling is just a scaling parameter 
        #we adjust empirically, in order to have some degree of control over how large are the angles.
        #angles is a vector of the form [base_angle, joint_angle]
        angles = scaling * np.linalg.solve(jacobian_matrix,error_vector)
        #Store current position as the previous position. To be used to get the difference in movement.
        previous_feature_point = feature_point
        #Move the robot by the amount specified in the angles vector
        tracking_failed, end_effector_bounding_box = move_and_track(tracker, vc, angles[0], angles[1], target_point, server, queue)
        if (tracking_failed):
            #reinitialize tracker if tracking has failed
            tracker, tracker_type = choose_tracking_method(2,minor_ver)
            rval = tracker.init(frame, end_effector_bounding_box.array_representation)
        #Position of the end effector after the move. (This is the current position of the end effector)
        feature_point = end_effector_bounding_box.centre
        #If safe mode is active and an obstacle was detected during the move: recompute initial jacobian, current position, error vector. Also, skip Broydens update 
        #for this iteration and solve the linear system at the begining of the loop. 
        # This is restarting visual servoing from scratch, with the last position of the end effector after avoiding the collision as starting point.
        if (safe_mode_active and obstacle_detected(queue)):
            print("Obstacle During Visual Servoing!")
            jacobian_matrix, feature_point = initial_jacobian(tracker, vc, feature_point, target_point, server, queue)
            error_vector = compute_delta(feature_point, target_point)
        else:
            #Broyden Update
            position_delta = compute_delta(previous_feature_point, feature_point)
            #Perform a rank 1 update of the jacobian
            jacobian_matrix = broyden_update(jacobian_matrix, position_delta , angles, alpha)
            #Compute the error between the current position of the end effector and the target
            error_vector = compute_delta(feature_point, target_point)
    server.sendTermination()
    print("Done")

