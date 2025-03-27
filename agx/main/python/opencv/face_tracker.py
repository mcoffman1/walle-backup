#!/usr/bin/env python

import rospy
import cv2
from imutils.video import VideoStream
import time


class FaceTracker:
    def __init__(self):
    
        #rospy.init_node('face_tracker_node')
        
        # Load the cascade classifier for detecting faces
        self.face_cascade = cv2.CascadeClassifier('/home/student/catkin_ws/src/opencv/haarcascades/haarcascade_frontalface_default.xml')
        
        # Open the video stream set res to (1920, 1080) or (1280, 720)
        self.cap = VideoStream(src=0, resolution=(1280, 720)).start()

        self.pcount=0
        
        self.display = False
    
        time.sleep(2)  # wait for the serial connection to establish
    
    def track_face(self):
    
        # Read the current frame from the video stream
        frame = self.cap.read()
        self.pcount += 1
    
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
        # Detect faces in the grayscale image
        # reduce scale to improve accuracy and slow prccessing (inc = .05)
        # increase min neighbors to reduce false poitives and miss some faces (inc = 5)
        # min size is min size face detected
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.15, minNeighbors=10, minSize=(30, 30))
    
        # Find the largest face in the frame
        largest_face = None
        max_area = 0
        for (x, y, w, h) in faces:
            area = w * h
            if area > max_area:
                largest_face = (x, y, w, h)
                max_area = area
    
        # Draw a rectangle around the largest face
        if largest_face is not None:
            (x, y, w, h) = largest_face
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
        # Display the frame with the largest face
        if self.display:
            cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('frame', 800, 600)
            cv2.imshow('frame', frame)
        
        # Send the coordinates of the largest face to the Arduino
        if largest_face is not None:
            (x, y, w, h) = largest_face
            # Normalize the coordinates
            x_center = int(x + w / 2)
            y_center = int(y + h / 2)
            return x_center, y_center
        else:
            return -1,-1
    
if __name__ == '__main__':
    try:
        ft = FaceTracker()
        while True:
            print(ft.track_face())
            time.sleep(.1)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        # Release the resources
        ft.cap.stop()
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass

