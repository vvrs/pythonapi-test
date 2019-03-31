
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist, PoseWithCovariance, TwistStamped
from nav_msgs.msg import Path
from dbw_mkz_msgs.msg import ThrottleCmd,BrakeCmd,SteeringCmd
from nav_msgs.msg import Odometry

from tf.transformations import quaternion_from_euler

import carla
import time
import random 
import sys
import threading

import matplotlib.pyplot as plt
import numpy as np
import time

import networkx as nx
import math

from get_topology import *

# plt.ion()
# plt.show()

class globalPathServer(object):
	"""Global is published everytime there is a request for global path over /get_global_path topic"""
	def __init__(self, world = " ", player = " ", ns = " ",source=0,destination=14):
		# super(GlobalPathServer, self).__init__()

		self.ns = ns
		# Get topology from the map
		_map = world.get_map()

		self.player = player 
		self.throttle = 0
		self.brake = 0
		self.steering = 0

		# Build waypoint graph
		topology,waypoints = get_topology(_map)
		self.graph,self.id_map = build_graph(topology)

		# get source and destination location from user (invoke while creating the object)
		self.source = source
		self.destination = destination

		self.p = get_shortest_path(self.graph, self.source, self.destination)


		# Because ROS Uses right handed coordinate system
		# and carla uses left handed coordinated system
		# https://math.stackexchange.com/questions/2626961/how-to-convert-a-right-handed-coordinate-system-to-left-handed
				
		# print "shorted path...",self.p 
		for i in range(len(self.p)):
			self.p[i][1] = -self.p[i][1]
		# print "shorted path...",self.p 
		# self.plot()
		rospy.init_node('{}_path_server'.format(self.ns), anonymous = True)
		rospy.Subscriber('{}/get_global_path'.format(self.ns), String, self.callback_update)
		rospy.Subscriber('carla/ThrottleCmd', ThrottleCmd, self.callback_throttle)
		rospy.Subscriber('carla/BrakeCmd', BrakeCmd, self.callback_brake)
		rospy.Subscriber('carla/SteeringCmd', SteeringCmd, self.callback_steering)

		self.path_publisher = rospy.Publisher('{}/global_path'.format(self.ns), Path, queue_size = 10)
		self.odom_publisher = rospy.Publisher('{}/odom'.format(self.ns), Odometry, queue_size = 10)
		self.speed_publisher = rospy.Publisher('{}/speed'.format(self.ns), TwistStamped, queue_size = 10)

		self.path = Path()
		self.makePathMessage()




	def makePathMessage(self):
		
		carla_path = self.p
		# posestamped required header to be set... 
		# Sequence Number - increase every time this function is called
		# time stamp - ros current time
		# frame_id - global
		
		poses = []
		for i in range(min(len(carla_path)-1,9000)):

			# posestamped required header to be set... 
			# Sequence Number - increase every time this function is called
			# time stamp - ros current time
			# frame_id - global
			posestamped = PoseStamped()
			
			pose = Pose()
			pose.position.x = carla_path[i][0]
			pose.position.y = carla_path[i][1]
			pose.position.z = carla_path[i][1]
			pose.orientation = self.directionFromTwoPointsQuaternion(carla_path[i],carla_path[i+1])

			posestamped.pose = pose 

			poses.append(posestamped)

		posestamped = PoseStamped()

		pose = Pose()
		pose.position.x = carla_path[-1][0]
		pose.position.y = carla_path[-1][1]
		pose.position.z = 0
		pose.orientation = self.directionFromTwoPointsQuaternion(carla_path[-2],carla_path[-1])
		posestamped.pose = pose

		poses.append(posestamped)

		self.path.poses = poses

	def callback_update(self, data):
		self.makePathMessage()
		self.path_publisher.publish(self.path)

	def callback_throttle(self, msg):
		self.throttle=msg.pedal_cmd
		if(self.throttle>0):
			self.brake = 0

	def callback_brake(self, msg):
		self.brake = msg.pedal_cmd
		if(self.brake>0):
			self.throttle = 0

	def callback_steering(self, msg):
		self.steering = - msg.steering_wheel_angle_cmd/8.2

	def apply_control(self):
		control_cmd = carla.VehicleControl(throttle=self.throttle,brake=self.brake,steer=self.steering)
		self.player.apply_control(control_cmd)

	def publish_odom(self,odom):
		out_odom = Odometry()
		out_odom.pose.pose.position.x = odom.location.x
		out_odom.pose.pose.position.y = -odom.location.y
		out_odom.pose.pose.position.z = odom.location.z

		quat = quaternion_from_euler(odom.rotation.roll/180*np.pi,odom.rotation.pitch/180*np.pi,-odom.rotation.yaw/180*np.pi)
		# quat = quaternion_from_euler(0,0,-odom.rotation.yaw/180*np.pi)
		# print(odom.rotation.yaw/180*np.pi)
		out_odom.pose.pose.orientation.x = quat[0]
		out_odom.pose.pose.orientation.y = quat[1]
		out_odom.pose.pose.orientation.z = quat[2]
		out_odom.pose.pose.orientation.w = quat[3]

		self.odom_publisher.publish(out_odom)

	def publish_path(self):
		self.path_publisher.publish(self.path)

	def publish_speed(self,speed):
		twist = TwistStamped()
		twist.twist.linear.x = speed
		self.speed_publisher.publish(twist)

	def plot(self):
		mapk = self.id_map.keys()
		srcind = self.id_map.values().index(self.source)
		destind = self.id_map.values().index(self.destination)
		source = mapk[srcind]
		dest = mapk[destind]

		plt.plot(source[0],source[1],'go--', linewidth=2, markersize=12)
		plt.plot(dest[0],dest[1],'ro--', linewidth=2, markersize=12)

		plt.plot(self.p[:,0],self.p[:,1])
		plt.draw()
		plt.pause(0.001)

	@staticmethod
	def  directionFromTwoPointsQuaternion(p1,p2):
		
		angle_ = np.arctan2(p2[1]-p1[1],p2[0]-p1[0]) 
		q = quaternion_from_euler(0, 0, angle_)
		quat_msg = Quaternion(q[0], q[1], q[2], q[3])
		return quat_msg

def main(argv):

	client = carla.Client('localhost',2000)
	client.set_timeout(2.0)

	world = client.get_world()

	
	# namespace - 'carla'
	node = globalPathServer(world,'carla')

	while not rospy.is_shutdown():
		rospy.spin()

if __name__ == '__main__':
	try:
		main(sys.argv[1:])
	except rospy.ROSInterruptException:
		pass