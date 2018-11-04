from client import Client
from ev3dev.ev3 import *
import socket
from time import sleep

motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)
clint = Client(9999)

while True:
    data = str(clint.pollData())
    if not motorA.is_running:
        motorA.position = 0
    if not motorB.is_running:
        motorB.position = 0
    if(data == 'exit'):
        break
    else:
        a,b = map(float,map(str,data.split('\t'))) 
        print(str(a)+ " " + str(b))
        a = int(a) % 360
        a = min([a, a - 360], key=lambda x: abs(x))
        b = int(b) % 360
        b = min([b, b - 360], key=lambda x: abs(x))
        print("After Processing: " + str(a)+ " " + str(b))
        sleep(15)
        motorA.run_to_rel_pos(position_sp=a, speed_sp = 50)
        motorB.run_to_rel_pos(position_sp=b, speed_sp = 50)
        sleep(5)
        motorA.stop()
        motorB.stop()