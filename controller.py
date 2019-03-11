#!/usr/bin/env python

from math import *
import numpy as np
import time
import sys


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from nav_msgs.msg import Path, Odometry
from tf.transformations import quaternion_from_euler
from ackermann_msgs.msg import AckermannDrive

import logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

global debug
debug = 1
class closestPoint:
	def __init__(self, points):
		self.points = points

	def distance(self,p1,p2):
		return sqrt((p2[0]-p1[0])**2+(p2[1]-p1[1])**2)

	def closest_node(self, node):
		nodes = self.points
		# print nodes
		print node
		# if debug:
		# 	print nodes
			# logging.debug("Points type: {}".format(type(nodes)))
			# logging.debug("Points array {0}\n".format(nodes.size()))
			# logging.debug("Node type: {0} content: {1} \n".format(type(node),node))
		dist_2 = np.sum((nodes - node)**2, axis=1)
		return np.argmin(dist_2)

class purePursuit:

	def __init__(self,points):

		rospy.loginfo("initializing pure pursuit node...")
		time.sleep(0.5)
		# rospy.init_node("purePursuit_node",anonymous=True)

		self.look_ahead_distance = 10

		self.target_speed = 5

		self.k = 0.1
		self.kp = 1

		self.init_xy = False
		self.init_heading = False

		self.car_init_x = None
		self.car_init_y = None 
		self.car_init_heading = None

		self.car_current_x = None
		self.car_current_y = None 
		self.car_current_heading = None
		self.current_speed = None
		
		# Get vehicle location from CARLA

		# Initialize vehicle controller message
		rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, self.callback_odom)


		self.cmd_pub = rospy.Publisher('/carla/ego_vehicle/ackermann_cmd', AckermannDrive, queue_size=1)

		self.points = points
		

	def callback_odom(self,data):
		
		self.car_current_x = data.pose.pose.position.x
		self.car_current_y = data.pose.pose.position.y

		self.current_speed = data.twist.linear.x

		quaternion = data.pose.orientation
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.car_current_heading = euler[2]


	def _controller(self):

		path = closestPoint(self.points)
		ind = path.closest_node((self.car_current_x,self.car_current_y))

		L = 0
		dind = ind
		print "current speed {0} m/s (from twist)".format(self.current_speed)
		Lf = self.k*self.current_speed + self.look_ahead_distance
		while L<Lf and dind < self.points.shape[0]-1:
			dx = self.points[dind+1][0] - self.points[dind][0]
			dy = self.points[dind+1][1] - self.points[dind][1]

			d = sqrt(dx**2 + dy**2)

			L += d

			dind += 1


		goal_x = self.points[(dind)%len(self.points)][0]
		goal_y = self.points[(dind)%len(self.points)][1]
		near_x = self.points[(ind)%len(self.points)][0]
		near_y = self.points[(ind)%len(self.points)][1]

		self.goalxpub.publish(goal_x)
		self.goalypub.publish(goal_y)

		distance = sqrt((self.car_current_x-goal_x)**2 + (self.car_current_y-goal_y)**2)
		distance2 = sqrt((near_x-goal_x)**2 + (near_y-goal_y)**2)

		print "Car current position :: ",self.car_current_x,self.car_current_y
		print "Goal position :: ",goal_x,goal_y
		print "Nearest point on the path:: ",near_x,near_y
		print "Distance to goal :: ",distance
		print "Look ahead distance:: ",distance2

		desired_pose = atan2((goal_y-self.car_current_y),(goal_x-self.car_current_x))

		print "desired_pose:: {0} ({1}) ".format(desired_pose,degrees(desired_pose))
		print "current heading:: {0} ({1}) ".format(self.car_current_heading,degrees(self.car_current_heading))
		error = (desired_pose - self.car_current_heading)

		print "heading error:: {0} ({1}) ".format(error,degrees(error))

		# if error < -3.14159:
		# 	# print "error is below -pi"
		# 	error += 2*3.14159
		# elif error > 3.14159:
		# 	# print "error is above pi"
		# 	error -= 2*3.14159
		# else:
		# 	# print "error is in between -pi and pi"
		# 	pass

		L = 2.6
		ld = 10
		kl = 1
		self.throttle = self._PIDControl(self.target_speed,self.current_speed)
		# self.throttle = min(self.throttle,0.2)
		# self.steering = np.arctan(2*L*sin(error)/(kl*self.throttle))
		self.steering = self.steering_ratio * np.arctan2(2*L*sin(error),Lf)
		# self.steering = error


	def _PIDControl(self,target, current):
		a = self.kp * (target - current)
		return a

	def publish_(self):

		self._controller()

		ai = self.throttle
		di = self.steering 

		print "throttle and steering angle :: ",ai,di
		print "\n\n"

		ackermann_cmd_msg = AckermannDrive()
		ackermann_cmd_msg.steering = di 
		ackermann_cmd_msg.speed = ai 

		self.cmd_pub.publish(ackermann_cmd_msg)
		# throttle_msg = make_throttle_msg(ai)
		# steering_msg = make_steering_msg(di)
		# brake_msg = make_brake_msg()
		# self.throttle_pub.publish(throttle_msg)
		# self.steering_pub.publish(steering_msg)
		# self.brake_pub.publish(brake_msg)

		rospy.on_shutdown(self.stopOnShutdown)
	def stopOnShutdown(self):
		ai = 0
		di = 0

		throttle_msg = make_throttle_msg(ai)
		steering_msg = make_steering_msg(di)
		brake_msg = make_brake_msg(1)
		self.throttle_pub.publish(throttle_msg)
		self.steering_pub.publish(steering_msg)
		self.brake_pub.publish(brake_msg)

		time.sleep(1)
		reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
		reset_world()
		# print "exiting..."
		rospy.signal_shutdown('Quit')
		# print "is shutdown :: ",rospy.is_shutdown()
		# return

def main():
	global points

	controller_object = purePursuit(points)

	r = rospy.Rate(50)

	while not rospy.is_shutdown():
		controller_object.publish_()
		r.sleep()

if __name__ == '__main__':
	main()