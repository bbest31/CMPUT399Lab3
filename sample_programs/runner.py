from client import Client
from ev3dev.ev3 import *
import socket

motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)
clint = Client(9999)

while True:
    data = str(clint.query())
    if not motorA.is_running:
        motorA.position = 0
    if not motorB.is_running:
        motorB.position = 0
    if(data == 'exit'):
        break
    else:
        a,b = map(float,map(str,data.split('\t'))) 
        motorA.run_to_rel_pos(position_sp=-a, speed_sp = 100)
        motorB.run_to_rel_pos(position_sp=b, speed_sp = 100)
