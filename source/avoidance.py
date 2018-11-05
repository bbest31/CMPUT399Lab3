#!/usr/bin/python3       
# RUN ON BRICK


from ev3dev.ev3 import *
import socket
from time import sleep


def avoid(base_rot, base_motor, joint_motor):
    coil(base_rot,base_motor, joint_motor)
    sleep(3)
    uncoil(base_rot,base_motor, joint_motor)


# Calculates the smallest angle of movement needed to get to the end effector coiled over the joint motor
def coil_joint(base_motor, joint_motor):
    joint_coil = 0
    rot_A = joint_motor.position % 360 
    rot_B = 360 - (joint_motor.position % 360)

    if(rot_A <= rot_B):
        joint_coil = 180 - rot_A
    else:
        joint_coil = -1*(180 - rot_B)
    return joint_coil


'''
This method initiates the coiling of the robot arm in to a position where we can
move to avoid an impeding obstacle.
'''
def coil(base_rot,base_motor, joint_motor):
    arm_angle = min(joint_motor.position % 360, 360 - (joint_motor.position % 360))
    
    if(base_rot < 0):
        # Robot base was moving in the negative rotation angle

        # Move the base away from the object
        # If the joint arm is close to 90 degrees angled then we don't need to move base as much
        if(arm_angle >= 70  and arm_angle <= 120):
            base_motor.run_to_rel_pos(position_sp=20, speed_sp = 50)
        elif(arm_angle <= -70  and arm_angle >= -120):
            base_motor.run_to_rel_pos(position_sp=20, speed_sp = 50)
        else:
            base_motor.run_to_rel_pos(position_sp=60, speed_sp = 50)

        angle = coil_joint(base_motor, joint_motor)
        joint_motor.run_to_rel_pos(position_sp=angle, speed_sp = 50)
        sleep(3)

    else:
        # Robot base was moving in the position rotation direction.

        # Move the base 90 degrees away from the object
        if(arm_angle >= 70  and arm_angle <= 120):
            base_motor.run_to_rel_pos(position_sp=-20, speed_sp = 50)
        elif(arm_angle <= -70  and arm_angle >= -120):
            base_motor.run_to_rel_pos(position_sp=-20, speed_sp = 50)
        else:
            base_motor.run_to_rel_pos(position_sp=-60, speed_sp = 50)

        angle = coil_joint(base_motor, joint_motor)
        joint_motor.run_to_rel_pos(position_sp=angle, speed_sp = 50)
        sleep(3)

    base_motor.stop()
    joint_motor.stop()


def uncoil(base_rot,base_motor, joint_motor):
    if(base_rot < 0):
        base_motor.run_to_rel_pos(position_sp=-150, speed_sp = 50)
        sleep(5)
        joint_motor.run_to_rel_pos(position_sp=180, speed_sp = 50)
        sleep(3)
    else :
        base_motor.run_to_rel_pos(position_sp=150, speed_sp = 50)
        sleep(5)
        joint_motor.run_to_rel_pos(position_sp=-180, speed_sp = 50)
        sleep(3)
