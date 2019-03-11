import sys
import carla
import time

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from nav_msgs.msg import Path

import numpy as np

import controller as cn 

class Navigation(object):
	"""docstring for Navigation"""
	def __init__(self,ns = " "):
		
		self.ns = ns
		self.points = []

		rospy.init_node('{}_navigation_node'.format(self.ns), anonymous = True)
		print '{}/global_path'.format(self.ns)
		self.global_path_pub = rospy.Publisher('/{}/get_global_path'.format(self.ns),String,queue_size = 10)
		time.sleep(0.5)
		# rospy.Subscriber('{}/global_path'.format(self.ns), Path, self.callback_gp)
		print "Subscriber started..."
			

		# self.path_publisher = rospy.Publisher('{}/global_path'.format(self.ns), Path, queue_size = 10)

	# def callback_gp(self,data):
	# 	print "callback_gp called..."
	# 	for i in data.poses:
	# 		self.points.append([i.pose.position.x,i.pose.position.y])
	def get_points(self):
		print "waiting for global path..."
		data = rospy.wait_for_message('{}/global_path'.format(self.ns), Path)
		
		for i in data.poses:
			self.points.append([i.pose.position.x,i.pose.position.y])
		self.global_path_pub.publish("abc")
		return np.array(self.points)

		
def main(argv):

	# namespace - 'carla'
	node = Navigation('carla')
	points = node.get_points()
	# print points
	controller_object = cn.purePursuit(points)
	r = rospy.Rate(50)

	while not rospy.is_shutdown():
		controller_object.publish_()
		r.sleep()

if __name__ == '__main__':
	try:
		main(sys.argv[1:])
	except rospy.ROSInterruptException:
		pass