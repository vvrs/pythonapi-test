
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler

import carla
import time
import random 
import sys

import matplotlib.pyplot as plt
import numpy as np
import time

import networkx as nx
import math

from get_topology import *


class globalPathServer(object):
	"""Global is published everytime there is a request for global path over /get_global_path topic"""
	def __init__(self, world = " ", ns = " ",source=0,destination=14):
		# super(GlobalPathServer, self).__init__()

		self.ns = ns
		# Get topology from the map
		_map = world.get_map()

		# Build waypoint graph
		topology,waypoints = get_topology(_map)
		graph,id_map = build_graph(topology)

		# get source and destination location from user (invoke while creating the object)
		self.source = source
		self.destination = destination

		self.p = get_shortest_path(graph, self.source, self.destination) 

		rospy.init_node('{}_path_server'.format(self.ns), anonymous = True)
		rospy.Subscriber('{}/get_global_path'.format(self.ns), String, self.callback_update)
		self.path_publisher = rospy.Publisher('{}/global_path'.format(self.ns), Path, queue_size = 10)

		self.path = Path()

	def makePathMessage(self):
		

		carla_path = self.p
		# posestamped required header to be set... 
		# Sequence Number - increase every time this function is called
		# time stamp - ros current time
		# frame_id - global
		
		poses = []
		for i in range(len(carla_path)-1):

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
		pose.position.z = 0;
		pose.orientation = self.directionFromTwoPointsQuaternion(carla_path[-1],carla_path[-1])
		posestamped.pose = pose

		poses.append(posestamped)

		self.path.poses = poses

	def callback_update(self, data):
		self.makePathMessage()
		self.path_publisher.publish(self.path)

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