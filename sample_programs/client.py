#!/usr/bin/python3       
# RUN ON BRICK
    
import socket
import os

class Client:
    def __init__(self,port):
        addr = os.getenv("SSH_CONNECTION").split() # [client_IP, client_port, server_IP, server_port] 
    
        host = "169.254.49.72"
        print("setting up client, address =", host, "port =", port)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.s.connect((host, port))                               
        

    def query(self):
        self.s.send("query".encode('UTF-8'))
        print('sending query')
        data = self.s.recv(128).decode("UTF-8")
        print('query received')
        return data
