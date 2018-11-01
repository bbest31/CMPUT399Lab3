#!/usr/bin/python
# RUN ON LAPTOP USING PYTHON 3.6

import socket

class Server:
    def __init__(self,host, port):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print('binding')
        self.s.bind((host,port))
        print('listening')
        self.s.listen(1)
        print('accepting')
        self.cs, address = self.s.accept()
        
        
    def answerQuery(self,data):
        print('receiving Query')
        self.cs.recv(16)
        print('sending answer')
        self.cs.send(data.encode("UTF-8"))

    #Place holder.
    #The only command/data we should send to the brick is a tuple (theta_1, theta_2) so that the robot can
    #Execute the movement. Ideally, after we send the command, we will listen (blocking) for a bit in order for the robot
    #To complete the command and report back to us.
    def sendData(self, base_angle, joint_angle):
        return "Implementation Pending"
