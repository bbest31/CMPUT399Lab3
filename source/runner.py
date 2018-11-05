from client import Client
from avoidance import *
from ev3dev.ev3 import *
import socket
from threading import Event, Thread
from time import sleep

base_motor = LargeMotor(OUTPUT_A)
joint_motor = LargeMotor(OUTPUT_B)
left_sensor = ColorSensor(INPUT_1)
right_sensor = ColorSensor(INPUT_2)
client = Client(9999)
joint_motor.reset()
base_motor.reset()
while True:
    data = str(client.pollData())
    if(data == 'EXIT'):
        base_motor.stop()
        joint_motor.stop()
        break
    else:
        base_angle, joint_angle = map(float,map(str,data.split('\t'))) 
        print(str(base_angle)+ " " + str(joint_angle))
        base_angle = int(base_angle) % 360
        base_angle = min([base_angle, base_angle - 360], key=lambda x: abs(x))
        joint_angle = int(joint_angle) % 360
        joint_angle = min([joint_angle, joint_angle - 360], key=lambda x: abs(x))
        print("After Processing: " + str(base_angle)+ " " + str(joint_angle))
        base_motor.run_to_rel_pos(position_sp=base_angle, speed_sp = 30)
        joint_motor.run_to_rel_pos(position_sp=joint_angle, speed_sp = 30)

        timer_thread = Thread(target=sleep, args=(5,))
        timer_thread.start()
        print("Wait 5 seconds")
        obstacle_evaded = False
        while timer_thread.is_alive():
            #Check sensor
            if(left_sensor.color == 5 or right_sensor.color == 5):
                print("Avoiding Obstacle")
                avoid(base_angle, base_motor, joint_motor)
                obstacle_evaded = True
        print("Done with 5 seconds")

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