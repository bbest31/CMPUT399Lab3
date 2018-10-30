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
