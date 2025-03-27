#!/usr/bin/env python

from math import sin, cos, pi, isnan

import rospy
import tf
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3
import tf.transformations as tr

class OdomPublisher:

	def __init__(self):

		rospy.init_node('ekf_odom_pub')
		
		# Initial pose
		self.initialX = 0.0
		self.initialY = 0.0
		self.initialTheta = 0.00000000001
		
		# Robot physical constants
		self.WHEEL_BASE = 0.6738 # Center of left tire to center of right tire
		self.TICKS_PER_METER = 690.5 # Original was 2800
		
		# Distance both wheels have traveled
		self.distanceLeft = 0
		self.distanceRight = 0

		self.current_time = rospy.Time.now()
		self.last_time = rospy.Time.now()

		# Create two new odomNew objects
		self.odomQuat = Odometry()
		self.odomEuler = Odometry()

		# Create odometry data publishers
		self.odom_data_pub_quat = rospy.Publisher("odom_data_quat", Odometry, queue_size=50)
		self.odom_data_pub_euler = rospy.Publisher("odom_data_euler", Odometry, queue_size=50)

		# Subscriber to ROS topics
		subForRightCounts = rospy.Subscriber("right_ticks", Int16, self.Calc_Right, queue_size=100, tcp_nodelay=True)
		subForLeftCounts = rospy.Subscriber("left_ticks", Int16, self.Calc_Left, queue_size=100, tcp_nodelay=True)
		subInitialPose = rospy.Subscriber("initial_2d", PoseStamped, self.set_initial_2d, queue_size=10)
		
		# Flag to see if initial pose has been received
		self.initialPoseRecieved = False
		
		self.lastCountL = 0
		self.lastCountR = 0
		self.dataLRecieved = False
		self.dataRRecieved = False

		self.loop_rate = rospy.Rate(60)


	# Get initial_2d message from either Rviz clicks or a manual pose publisher
	def set_initial_2d(self, rvizClick):
		self.initialX = rvizClick.pose.position.x
		self.initialY = rvizClick.pose.position.y
		self.initialTheta = rvizClick.pose.orientation.z
		self.initialPoseRecieved = True
		
		
	def updateEulerXYT(self):
		self.initialX = odomQuat.pose.pose.position.x
		self.initialY = odomQuat.pose.pose.position.y
		quat = (
			self.odomQuat.pose.pose.orientation.x,
			self.odomQuat.pose.pose.orientation.y,
			self.odomQuat.pose.pose.orientation.z,
			self.odomQuat.pose.pose.orientation.w)
		roll, pitch, yaw = tr.euler_from_quaternion(quat)
		self.initialTheta = yaw
		


	# Calculate the distance the left wheel has traveled since the last cycle
	def Calc_Left(self, leftCount):
		if leftCount.data != 0 and self.lastCountL != 0:
			leftTicks = (leftCount.data - self.lastCountL)

			if leftTicks > 10000:
				leftTicks = 0 - (65535 - leftTicks)
			elif leftTicks < -10000:
				leftTicks = 65535-leftTicks
			else:
				pass
			self.distanceLeft = leftTicks/self.TICKS_PER_METER
		self.lastCountL = leftCount.data
		self.dataLRecieved = True


	# Calculate the distance the right wheel has traveled since the last cycle
	def Calc_Right(self, rightCount):

		if rightCount.data != 0 and self.lastCountR != 0:
			rightTicks = rightCount.data - self.lastCountR

			if rightTicks > 10000:
				rightTicks = 0 - (65535 - rightTicks)
			elif rightTicks < -10000:
				rightTicks = 65535 - rightTicks
			else:
				pass
			self.distanceRight = rightTicks/self.TICKS_PER_METER
		self.lastCountR = rightCount.data
		self.dataRRecieved = True



	# Update odometry info
	def update_odom(self):

		self.current_time = rospy.Time.now()
		timediff = self.current_time - self.last_time
		dt = timediff.to_sec()
		
		dis = (self.distanceRight + self.distanceLeft)/2
		dth = (self.distanceRight - self.distanceLeft) / self.WHEEL_BASE
		avgth = self.initialTheta + (dth/2)
		dx = dis*cos(avgth)
		dy = dis*sin(avgth) 
		vx = dis/dt
		vth = dth/dt
  
		self.initialX = self.initialX + dx
		self.initialY = self.initialY + dy
		self.initialTheta = self.initialTheta + dth
  
		if self.initialTheta > 2*pi:
			self.initialTheta = self.initialTheta - 2*pi
		elif self.initialTheta < -2*pi:
			self.initialTheta = self.initialTheta + 2*pi
		else:
			pass

		# since all odometry is 6DOF we'll need a quaternion created from yaw
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.initialTheta)
  
		# next, we'll publish the odometry message over ROS
		self.odomQuat.header.stamp = self.current_time
		self.odomQuat.header.frame_id = "odomQuat"
		self.odomQuat.child_frame_id = "base_link"	
		
		# Prevent lockup from a single bad cycle
		if isnan(self.odomQuat.pose.pose.position.x) or isnan(self.odomQuat.pose.pose.position.y) or isnan(self.odomQuat.pose.pose.position.z):
			print("isnan failed: Did not update pose")
		else:
			# set the position
			self.odomQuat.pose.pose = Pose(Point(self.initialX, self.initialY, 0), Quaternion(*odom_quat))

			# set the velocity
			self.odomQuat.twist.twist = Twist(Vector3(vx, 0, 0), Vector3(0, 0, vth))
		
		# Save the pose data for the next cycle
		#print("  X: {},  Y: {},  Theta: {}".format(self.initialX,self.initialY,self.initialTheta))

		for i in range(36):
			if i == 0 or i == 7 or i == 14:
				self.odomQuat.pose.covariance[i] = 0.01
			elif i == 21 or i == 28 or i == 35:
				self.odomQuat.pose.covariance[i] += 0.1
			else:
				self.odomQuat.pose.covariance[i] = 0
		
		self.odom_data_pub_quat.publish(self.odomQuat)
		
		self.last_time=self.current_time
  
		# Publish the odometry message
		self.dataLRecieved = False
		self.dataRRecieved = False


if __name__ == '__main__':
	try:
		op = OdomPublisher()
		while not rospy.is_shutdown():
			if op.initialPoseRecieved and op.dataLRecieved and op.dataRRecieved:
				op.update_odom()
			op.loop_rate.sleep()
	except rospy.ROSInterruptException:
		pass
