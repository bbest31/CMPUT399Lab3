#!/usr/bin/python3       
# RUN ON BRICK
    
import socket
import os

class Client:
    def __init__(self,port):
        
        #We need to use the ipv4 address that shows up in ipconfig in the computer

        host = "169.254.222.14"

        print("setting up client, address =", host, "port =", port)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.s.connect((host, port))                               
        

    def pollData(self):
        print("Waiting for Data")
        data = self.s.recv(128).decode("UTF-8")
        print("Data Received")
        return data
    
    def sendAcknowledgement(self):
        self.s.send("DONE".encode("UTF-8"))

    def sendReset(self):
        self.s.send("RESET".encode("UTF-8"))
