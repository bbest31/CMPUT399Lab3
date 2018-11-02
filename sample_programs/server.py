#!/usr/bin/python
# RUN ON LAPTOP USING PYTHON 3.6

import socket

class Server:
    def __init__(self,host, port):
       # setup server socket
        serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        host = socket.gethostname()   
        print ("host: ", host)                        
        port = 9999
        serversocket.bind((host, port))                                  
        # queue up to 5 requests
        serversocket.listen(5) 
        self.cs,addr = serversocket.accept()  
        print ("Connected to: " +str(addr) )
        
        
    def answerQuery(self,data):
        print('sending answer')
        self.cs.send(data.encode("UTF-8"))
        print('receiving Query')
        self.cs.recv(16)

    #The only command/data we should send to the brick is a tuple (theta_1, theta_2) so that the robot can
    #Execute the movement. Ideally, after we send the command, we will listen (blocking) for a bit in order for the robot
    #To complete the command and report back to us.
    def sendData(self, base_angle, joint_angle):
        #Format in which the client expects the data
        # angle1    angle2
        self.answerQuery(str(base_angle)+"\t"+str(joint_angle))
