#!/usr/bin/python3       
# RUN ON BRICK
    
import socket

def setup_client(host, port):
	# create a socket object
	print("setting up client")
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
	# connect to hostname on the port.
	s.connect((host, port))                               
	# Receive no more than 1024 bytes
	while True:
		msg = s.recv(1024) 
		if len(msg) > 0:
			print("Received: ", msg.decode('utf-8'))
		else:
			break
	s.close()

if __name__ == "__main__":
	host = socket.gethostname()                           
	port = 9999
	setup_client(host, port)