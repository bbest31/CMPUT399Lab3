#!/usr/bin/python3       
# RUN ON BRICK
    
import socket
import os

class Client:
    def __init__(self,port):
        addr = getenv("SSH_CONNECTION").split() # [client_IP, client_port, server_IP, server_port] 
    
        host = addr[0]
        print("setting up client")
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.s.connect((host, port))                               
        

    def query(self):
        self.s.send(b"query")
        self.s.recv(128)
