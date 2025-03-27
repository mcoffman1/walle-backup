
import cv2 as cv 
import numpy as np
from imutils.video import VideoStream
import imutils

def canny_webcam():
    
	#cap = cv.VideoCapture(0)
	cap = VideoStream(src=4).start()
	#cap.set(3,600)
	#cap.set(4,400)
	edgeshow = -1

	while True:
		if cv.waitKey(20) == ord(' '):
			edgeshow=edgeshow*-1
			
		#ret, frame = cap.read()  
		frame = cap.read()

		edge = cv.Canny(frame, 75, 100)#high threshold
		#edge = cv.Canny(frame, 5, 45)# low threshhold

		if edgeshow==1:  
			cv.imshow('Canny Edge', edge)
		else:
			cv.imshow('Normal', frame)

		if cv.waitKey(20) == ord('q'):  
			break

canny_webcam()
