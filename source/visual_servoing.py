import cv2
import sys
import numpy as np
from server import Server
from rectangle import Rectangle
from threading import Event, Thread
from time import sleep
from queue import Queue

#This procesude cointains the routine for visual servoing.

#Choosing tracking method out all of the possible implementations provided by the OpenCV contrib module
#This will return an uninitialized tracker object of the type of our choosing.
#We decided to go for the KCF tracker in our visual servoing implementation since it was the one that we thought
#had the best performance.
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

#Computes the delta of two vectors, where each vector is an integer tuple in image space (u,v)
def compute_delta(initial_vector, final_vector):
    return (final_vector[0] - initial_vector[0], final_vector[1] - initial_vector[1])

#This method will return true if the client (Brick) has sent a message to the server letting it know
#that an obstacle was detected and avoided
#Input: queue [Thread-safe Queue]: Mutable data structure to store (and return) the messages received from the client
#Output: [Boolean] True if an obstacle has been detected, false otherwise
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


#This function will issue a move command to the robot by the specified base and joint angles.
#It will keep track of the position of both, the end effector and the target point while the movement is being executed
#Once the robot sends a message letting the server know that the movement is complete (either DONE or RESET) this function will terminate.
#Inputs: tracker [Mutable tracker]: Reference to the end effector tracker object, for the purposes of updating the tracker
#               during the moves of the initial jacobian computation
#        vc [OpenCV videocapture]: Reference to the webcam
#        base_angle [Float]: angle in degrees by which we want the base to rotate
#        joint_angle [Float]: angle in degrees by which we want the joint to rotate
#        target_tracker [Mutable tracker]: Reference to the target point tracker object, for the purposes of updating the tracker
#               during the moves of the initial jacobian computation
#        server [Server object]: Reference to an instance of the server class so that movement orders can be issues and messages can be recived
#        queue [Thread-safe Queue]: Mutable data structure to store (and return) the messages received from the client
#Outputs: A triple consisteng of the tracking_failed [Boolean], which will be set to true if we lost the tracking of the end effector.
#         bounding_rectangle [Rectangle object]  which contains a representation of the end effector bounding box
#         target_bounding_rectangle [Rectangle object]  which contains a representation of the target object bounding box              
def move_and_track(tracker, vc, base_angle, joint_angle, target_tracker, server, queue):
    #Send move command to the server. Start as a new thread, pass the queue object to the thread
    #in order to be able to retrieve messages from client (Brick) in the main thread
    robot_movement_thread = Thread(target=server.sendAngles, args=(base_angle,joint_angle, queue))
    robot_movement_thread.start() 
    tracking_failed = False
    #While the thread is alive, we know that the server is still waiting to receive a message from the client letting it know that is done moving.
    while robot_movement_thread.is_alive():
        #Read get image from webcam and update trackers and bounding boxes while the robot is moving
        rval, frame = vc.read()
        ok, bbox = tracker.update(frame)
        target_ok, tbbox = target_tracker.update(frame)
        target_bounding_rectangle = Rectangle(tbbox)
        bounding_rectangle = Rectangle(bbox)
        if (ok and target_ok):
            # Tracking success
            #Update the current position of the end effector
            current_position = bounding_rectangle.centre 
            #Update the current position of the target point
            target_point = target_bounding_rectangle.centre
            #Draw rectangles representing the bounding box arond the end effector. The centroid of the end effector bounding box and a small rectangle representing the 
            #tracked target point
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

    #If we lost tracking and the KCF tracker couldn't recover, we need the user to manually select the end effector and the target point again
    if tracking_failed:
        bounding_rectangle, target_bounding_rectangle = select_tracked_regions(vc)

    return tracking_failed, bounding_rectangle, target_bounding_rectangle


#This is an implementation of what we have in http://ugweb.cs.ualberta.ca/~vis/courses/robotics/lectures/lec10VisServ.pdf (pages 25 and 26)
#For the Broyden Update (Online Jacobian computation)
#Input: jacobian_matrix [Numpy Matrix]: Jacobian Matrix at current iteration
#       position_delta [Numpy array]: Change in position from last movement
#       angle_delta [Numpy array]: Amount (in angles) by which each of the motors rotated in the last movement. Array is of the form [base_angle, joint_angle]
#       alpha [Float]: Amount to by which we want to scale the update
#Output: [Numpy matrix] that corrsponds to the updated Jacobian
def broyden_update(jacobian_matrix, position_delta, angle_delta, alpha):
    #We need to make sure these are numpy objects.
    position_delta = np.array(position_delta)
    angle_delta = np.array(angle_delta)
    jacobian_matrix = np.mat(jacobian_matrix)
    #Compute the fraction term separately (for clarity)
    numerator = np.outer(position_delta - jacobian_matrix.dot(angle_delta),angle_vector)
    denominator = angle_delta.dot(angle_delta)
    #This will essentially divide the 2x2 matrix in the numerator
    #By the squared norm of the the angle_delta
    fraction = numerator/denominator
    #Perform the rank 1 update. This will return an updated jacobian matrix as a numpy matrix
    return  jacobian_matrix + (alpha * fraction)

#Returns initial jacobian and postion of the end effector after the computation
#of the initial Jacobian
#Inputs: tracker [Mutable tracker]: Reference to the end effector tracker object, for the purposes of updating the tracker
#               during the moves of the initial jacobian computation
#        vc [OpenCV videocapture]: Reference to the webcam
#        previous_position [Int tuple]: Tuple of integers (u,v) representing the position of the end effector at the time
#               the function was called
#        target_tracker [Mutable tracker]: Reference to the target point tracker object, for the purposes of updating the tracker
#               during the moves of the initial jacobian computation
#        server [Server object]: Reference to an instance of the server class so that movement orders can be issues and messages can be recived
#        queue [Thread-safe Queue]: Mutable data structure to store (and return) the messages received from the client
#Output: A tuple consisting of the initial jacobian (Numpy matrix) and the position of the end effector after the computation ((u,v)  integer tuple
def initial_jacobian(tracker, vc, previous_position, target_tracker, server, queue):
    #Initial angles
    base_angle = 20
    joint_angle = 40
    #Calculate first column of the initial Jacobian
    #This is done by moving the base and fixing the joint
    tracking_failed, end_effector_bounding_box, target_bounding_box = move_and_track(tracker, vc, base_angle, 0, target_tracker, server, queue)
    current_position = end_effector_bounding_box.centre
    position_delta = compute_delta(previous_position, current_position)
    jacobian_column_1 = [position_delta[0] / base_angle, position_delta[1] / base_angle]
    #Update the previous position to be the current position of the end effector.
    previous_position = current_position
    #Calculate second column of the initial Jacobian
    #This is done by moving the joint and fixing the base
    tracking_failed, end_effector_bounding_box, target_bounding_box = move_and_track(tracker, vc, 0, joint_angle, target_tracker, server, queue)
    current_position = end_effector_bounding_box.centre
    position_delta = compute_delta(previous_position, current_position)
    jacobian_column_2 = [position_delta[0] / joint_angle, position_delta[1] / joint_angle]
    #Once the two columns are computed:
    #Join the two columns and transpose them (they were stored as row vectors)
    jacobian_matrix = np.mat((jacobian_column_1, jacobian_column_2)).getT()
    print(jacobian_matrix)
    return jacobian_matrix, current_position

#Takes a videocapture object and uses it to prompt the user to define bounding boxes for both the end effector and the target point
#Input:  vc [OpenCV videocapture]: Reference to the webcam
#Outputs: A tuple consisting of bounding_rectangle [Rectangle object]  which contains a representation of the end effector bounding box
#         and target_bounding_rectangle [Rectangle object]  which contains a representation of the target object bounding box    
def select_tracked_regions(vc):
    #Read frame
    rval, frame = vc.read()
    cv2.putText(frame, "Select End Effector", (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 127, 0), 2)
    #In here the user draws a bounding box around the end effector.
    #bbox is an array represending a rectangle: [x, y, height, width]
    bbox = cv2.selectROI("webcam",frame, False)
    #Convert the bounding box into a Rectangle object to perform automatic calculation of the four corners as well as the centre point
    bounding_rectangle = Rectangle(bbox)
    #Display feature point position and coordinates in the next frame
    rval, frame = vc.read()
    cv2.rectangle(frame, bounding_rectangle.top_left, bounding_rectangle.bottom_right, (255,0,0), 2, 1)
    cv2.rectangle(frame, (int(feature_point[0]) - 2, int(feature_point[1]) - 2), (int(feature_point[0]) + 2,  int(feature_point[1]) + 2), (0, 0, 0), -1) 
    cv2.putText(frame, "Feature Point (x,y): "  + str(feature_point), (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
    #Display prompt to select target point
    cv2.putText(frame, "Select Target Point", (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 127, 0), 2)
    target_bbox = cv2.selectROI("webcam", frame, False)
    #Convert the target bounding box into a Rectangle object to perform automatic calculation of the four corners as well as the centre point
    target_bounding_rectangle = Rectangle(target_bbox)
    return bounding_rectangle, target_bounding_rectangle


if __name__ == '__main__' :
    #Get OpenCV version
    (major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
    #Start listening on port 9999
    server = Server(9999)
    #Thread-safe queue to get data from threads
    queue = Queue()
    #Set safety mode (Obstacle detection and avoidance)
    safe_mode_active = True

    #Enable or disable safety mode on the client (Brick)
    if (safe_mode_active):
        server.sendEnableSafetyMode()
    else:
        server.sendDisableSafetyMode()

    ##### Instantiate KCF Trackers ######
    #End effector KCF Tracker
    tracker, tracker_type = choose_tracking_method(2,minor_ver)
    #Target Point/Object KCF tracker
    target_tracker, target_tracker_type = choose_tracking_method(2,minor_ver)
    
    #Initialize webcam
    vc = cv2.VideoCapture(1)

    #Get the first frame
    if vc.isOpened(): 
        rval, frame = vc.read()
    else:
        rval = False

    #This loop makes sure that the camera has time to properly initialize and adjust to the
    #lighting conditions. If this loop is not present, then the frame object which is used 
    #To specify the regions of interest further down the code would be too dark
    i = 0
    while rval and i < 100:
        # Read a new frame
        rval, frame = vc.read()
        i = i + 1
    
    #In here if rval is false we could throw an exception.

    cv2.namedWindow("webcam")

    #Select draw bounding boxes around end effector and target point
    bounding_rectangle, target_bounding_rectangle = select_tracked_regions(vc)

    #Centre point of the end effector bounding box, is the point that we are using for the minimization of error and the point at which 
    #We consider the end effector to be.
    feature_point = bounding_rectangle.centre 
    #Centre point of the target object bounding box, is the point that we are using for the minimization of error and the point at which 
    #We consider the target to be.
    target_point = target_bounding_rectangle.centre

    # Initialize end effector KCF tracker with the corresponding end effector bounding box
    rval = tracker.init(frame, bounding_rectangle.array_representation)
    # Initialize target point KCF tracker with the corresponding end target bounding box
    tval = target_tracker.init(frame, target_bounding_rectangle.array_representation)

    ###############Estimate Initial Jacobian################################

    jacobian_matrix, feature_point = initial_jacobian(tracker, vc, feature_point, target_tracker, server, queue)
    #If safe mode is active and an obstacle was detected during the computation, recompute the initial Jacobian
    if (safe_mode_active and obstacle_detected(queue)):
        print("Obstacle During Initial Jacobian!")
        jacobian_matrix, feature_point = initial_jacobian(tracker, vc, feature_point, target_tracker, server, queue)

    #############End of Initial Jacobian Estimation#########################
    

    #############Start Visual Servoing##########################
    #With the target point set, and the initial jacobian calculated, we can 
    #now start the actual process for visual servoing.

    #Initial Error
    error_vector = compute_delta(feature_point, target_point)
    #Constants for scaling the results (as shown in the last lab)
    alpha = 0.5
    scaling = 0.4

    #This loop mimics the process outlined in http://ugweb.cs.ualberta.ca/~vis/courses/robotics/lectures/lec10VisServ.pdf page 26
    while np.linalg.norm(error_vector) > 10:
        #Solve the linear system e = J*q -> q = scaling * (inverse(J)*e). Where scaling is just a scaling parameter 
        #we adjust empirically, in order to have some degree of control over how large are the angles.
        #angles is a vector of the form [base_angle, joint_angle]
        angles = scaling * np.linalg.solve(jacobian_matrix,error_vector)
        #Store current position as the previous position. To be used to get the difference in movement.
        previous_feature_point = feature_point
        #Move the robot by the amount specified in the angles vector
        tracking_failed, end_effector_bounding_box, target_bounding_box = move_and_track(tracker, vc, angles[0], angles[1], target_tracker, server, queue)
        if (tracking_failed):
            #reinitialize tracker if tracking has failed
            tracker, tracker_type = choose_tracking_method(2,minor_ver)
            rval = tracker.init(frame, end_effector_bounding_box.array_representation)
            tval = target_tracker.init(frame, target_bounding_box.array_representation)
        #Position of the end effector after the move. (This is the current position of the end effector)
        feature_point = end_effector_bounding_box.centre
        #Update position of the target point based on latest information from tracking
        target_point = target_bounding_box.centre
        #If safe mode is active and an obstacle was detected during the move: recompute initial jacobian, current position, error vector. Also, skip Broydens update 
        #for this iteration and solve the linear system at the begining of the loop. 
        # This essentially is restarting visual servoing from scratch, with the last position of the end effector after avoiding the collision as starting point.
        if (safe_mode_active and obstacle_detected(queue)):
            print("Obstacle During Visual Servoing!")
            jacobian_matrix, feature_point = initial_jacobian(tracker, target_tracker, vc, feature_point, target_point, server, queue)
            error_vector = compute_delta(feature_point, target_point)
        else:
            #Broyden Update
            position_delta = compute_delta(previous_feature_point, feature_point)
            #Perform a rank 1 update of the jacobian
            jacobian_matrix = broyden_update(jacobian_matrix, position_delta , angles, alpha)
            #Compute the error between the current position of the end effector and the target
            error_vector = compute_delta(feature_point, target_point)
    #Send message to the client to let it know that we are done.
    server.sendTermination()
    print("Done")

