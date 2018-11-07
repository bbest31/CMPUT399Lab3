import cv2
import sys
import numpy as np
from server import Server
from rectangle import Rectangle
from threading import Event, Thread
from time import sleep
from queue import Queue
from static_visual_servoing_methods import *


if __name__ == '__main__' :S

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

    #End effector KCF Tracker
    tracker, tracker_type = choose_tracking_method(2,minor_ver)
    
    #Initialize webcam
    vc = cv2.VideoCapture(0)

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
        tracking_failed, end_effector_bounding_box = move_and_track(tracker, vc, angles[0], angles[1], target_point, server, queue)
        if (tracking_failed):
            #reinitialize tracker if tracking has failed
            rval, frame = vc.read()
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

