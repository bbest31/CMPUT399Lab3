#!/usr/bin/python3       
# RUN ON BRICK
    
import socket
import os

#This class handles the client side of communication. It has a set of predefined messages to send to the server
#as well as functionality to poll and decode data.

class Client:
    def __init__(self,port):
        
        #We need to use the ipv4 address that shows up in ipconfig in the computer for the USB
        #Ethernet adapter handling the connection to the EV3
        host = "169.254.105.60"
        print("setting up client, address =", host, "port =", port)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.s.connect((host, port))                               
        
    #Block until a message from the server is received. When the message is received it will be decoded
    #and returned as a string.
    #Output: UTF-8 decoded string containing the instructions from server.
    def pollData(self):
        print("Waiting for Data")
        data = self.s.recv(128).decode("UTF-8")
        print("Data Received")
        return data
    
    #Sends a message to the server letting it know that the movement of the motors was executed without
    #any inconvenience.
    def sendDone(self):
        self.s.send("DONE".encode("UTF-8"))

    #Sends a message to the server letting it know that there was an isse during the execution of the movement (obstacle avoided)
    #and that the initial jacobian should be recomputed (Visual servoing started from scratch)
    def sendReset(self):
        self.s.send("RESET".encode("UTF-8"))
