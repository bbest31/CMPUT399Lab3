from client import Client
from avoidance import *
from ev3dev.ev3 import *
import socket
from threading import Event, Thread
from time import sleep

#This routine will run on the lego Brick. It initalizes the client and will listen 
#for commands from the server.

#Instantiate motors and sensors
base_motor = LargeMotor(OUTPUT_A)
joint_motor = LargeMotor(OUTPUT_B)
left_sensor = ColorSensor(INPUT_1)
right_sensor = ColorSensor(INPUT_2)

#Attempt to connect to server
client = Client(9999)
#Reset motors to make sure the encoder counter starts at 0 for both.
joint_motor.reset()
base_motor.reset()

#By default safety mode is off on the brick
safety_mode_on = False
while True:
    #Block until a command/message from the server is received
    data = str(client.pollData())
    if(data == 'EXIT'):
        #Terminate the routine on the client
        base_motor.stop()
        joint_motor.stop()
        break
    elif(data == "SAFETY_ON"):
        #Message from server enabling safety mode
        safety_mode_on = True
    elif(data == "SAFETY_OFF"):
        #Message from server disabling safety mode
        safety_mode_on = False
    else:
        #If the data we got from the server is not one of the predifined messages
        #We assume (big assumption) that the message will contain the angles in the 
        #Agreed format: base_angle      joint_angle
        #We convert the angles to float after splitting the message on the tab
        base_angle, joint_angle = map(float,map(str,data.split('\t'))) 
        #This will make sure that both angles are at most 360 degrees. In addition,
        #it will consider both, the clockwise and counterclockwise rotations to reach
        #the desired position for each motor and will pick the smallest one. So that the chances of making
        #very large rotations are lessened.
        base_angle = int(base_angle) % 360  
        base_angle = min([base_angle, base_angle - 360], key=lambda x: abs(x))
        joint_angle = int(joint_angle) % 360
        joint_angle = min([joint_angle, joint_angle - 360], key=lambda x: abs(x))
        #After the proper rotation angle has been selected, we send the move command to the motors
        print("Moving Base: " + str(base_angle)+ " Moving Joint: " + str(joint_angle))
        base_motor.run_to_rel_pos(position_sp=base_angle, speed_sp = 30)
        joint_motor.run_to_rel_pos(position_sp=joint_angle, speed_sp = 30)
        #We will use a timer thread and assume (big assumption) that the motors will complete their moves
        #within 3 seconds. The reason we are using a timer thread is that we tried to use the ev3 motor api (motor.wait_while('running'), motor.wait_until_not_moving(), etc)
        #but the results were not satisfactory since the loop would block indefenitely after the first move. Seeing that time was a big concern at that point, we decided
        #to make the compromise and use the timer thread.
        timer_thread = Thread(target=sleep, args=(3,))
        timer_thread.start()
        obstacle_evaded = False
        #While the motors are moving
        while timer_thread.is_alive():
            #If the safety mode is in, we check the sensors for obstacles. We assume obstacles will be red objects.
            if(safety_mode_on  and (left_sensor.color == 5 or right_sensor.color == 5)):
                #Collision avoidance can be thought as a "reflex" of the robot. If an obstacle is detected, the avoid routine will take over
                #ignoring any previous commands. It will attempt to avoid the obstacle using coiling (see report).
                print("Avoiding Obstacle")
                avoid(base_angle, base_motor, joint_motor)
                #Once the avoidance routine is complete. It will set the obstacle_evaded variable to true.
                obstacle_evaded = True
            else:
                pass

        base_motor.stop()
        joint_motor.stop()
        if (obstacle_evaded):
            #If an obstacle was evaded we send a message to the server
            #to let it know that it should recompute the initial jacobian
            #and start the visual servoing from scratch at the resulting location
            client.sendReset()
        else:
            #If no obstacle was detected, we assume that the movement was executed without
            #any issues. We send a message to the server letting it know that the robot is done
            #moving and that it can update the jacobian matrix using the current position of the
            #end effector.
            client.sendDone()