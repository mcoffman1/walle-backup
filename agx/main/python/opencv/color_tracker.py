from collections import deque
from imutils.video import VideoStream
from threading import Thread
import numpy as np
import argparse
import cv2
import imutils
import time
import serial
import struct
import range_finder

# user input to determine if range finder should run
answer = input('Do you want to run Range Finder? (y/n): ')

if answer == 'y' or answer == 'Y':
	minmaxvalues = (range_finder.main())
else:
# set default values here
	minmaxvalues = [5, 160, 125, 18, 216, 245]

# Create separate thread for sending data
def send_data():
	# coms to arduino
	ser = serial.Serial('com16', 9600)
	lastx = 0
	lasty = 0

	while True:
		if lastx == x or lasty == y:
			time.sleep(.1)
		else:
			# print center relative to center of screen
			# then calculate Delta x and y based on ploynomial regression
			relx = int(300 - x)
			rely = int(225 - y)
			#print(delx,dely)
			delx = .00000001247*relx**3+0*relx**2+.0008753*relx-0
			dely = .00000001247*rely**3+0*rely**2+.0008753*rely-0
			delx = round(delx,3)
			dely = round(dely,3)
			# send coordinates to arduino
			mydata = 'X{} Y{} '.format(x,y)
			print(mydata)
			mydata = mydata.encode('utf-8')
			ser.write(mydata)
			#ser.write(b'\n')
			lastx = x
			lasty = y

# x, y coord need to be global to pass them between threads
x = 0
y = 0

#serialthread = Thread(target=send_data)
#serialthread.daemon=True
#serialthread.start()

# parse arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64, help="max buffer size")
args = vars(ap.parse_args())

#================= LOWER AND UPPER HSV ==================

lower = (minmaxvalues[0], minmaxvalues[1], minmaxvalues[2])
upper = (minmaxvalues[3], minmaxvalues[4], minmaxvalues[5])
pts = deque(maxlen=args["buffer"])

# if a video path was not supplied, grab the reference
if not args.get("video", False):
	vs = VideoStream(src=0).start()

# otherwise, grab a reference to the video file
else:
	vs = cv2.VideoCapture(args["video"])

# allow the camera or video file to warm up
time.sleep(2.0)

# loop until q is pressed


while True:

	# grab the current frame
	frame = vs.read()

	# handle the frame from VideoCapture or VideoStream
	frame = frame[1] if args.get("video", False) else frame

	# if we are viewing a video and we did not grab a frame, then we have reached the end of the video
	if frame is None:
		break

	# resize the frame, blur it, and convert it to the HSVcolor space
	frame = imutils.resize(frame, width=600)
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	# construct a mask for the color, then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, lower, upper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

#=========================CONTOURS========================================

    # find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


        # only proceed if the radius meets a minimum size
		if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius), (255, 0, 0), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)

	# update the points queue
	pts.appendleft(center)

#============================DRAW TRAIL================================

# loop over the set of tracked points
	for i in range(1, len(pts)):
		# if either of the tracked points are None, ignore
		# them
		if pts[i - 1] is None or pts[i] is None:
			continue

		# otherwise, compute the thickness of the line and
		# draw the connecting lines
		thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
		cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

	# show the frame to our screen
	cv2.imshow("Frame", frame)
	key = cv2.waitKey(1) & 0xFF

	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break

# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
	vs.stop()

# otherwise, release the camera
else:
	vs.release()

# close all windows
cv2.destroyAllWindows()