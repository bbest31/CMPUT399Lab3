#!/usr/bin/python
# RUN ON LAPTOP USING PYTHON 3.6

import socket                                         
import cv2

def capture_frames(clientsocket):
	cv2.namedWindow("webcam")
	vc = cv2.VideoCapture(0)
	if vc.isOpened(): # try to get the first frame
		rval, frame = vc.read()
	else:
		rval = False
	i = 0
	while rval:
		# handle current frame
		cv2.imshow("webcam", frame)
		rval, frame = vc.read()
		# send calculated information to ev3
		clientsocket.send(str(i).encode('utf-8'))
		i+=1
		# check if esc key pressed
		key = cv2.waitKey(20)
		if key == 27:
			clientsocket.close()
			cv2.destroyWindow("webcam")
			break

if __name__ == "__main__":
	# setup server socket
	serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
	host = socket.gethostname()   
	print ("host: ", host)                        
	port = 9999
	serversocket.bind((host, port))                                  
	# queue up to 5 requests
	serversocket.listen(5) 
	clientsocket,addr = serversocket.accept()  
	print ("Connected to: " +str(addr) )
	capture_frames(clientsocket)
