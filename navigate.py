import sys
import carla
import time

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist
from nav_msgs.msg import Path,Odometry

import numpy as np

import controller as cn 
import matplotlib.pyplot as plt 
plt.ion()
plt.show()

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

class DynamicUpdate():
	def __init__(self,points):
		global i
		self.xdata=[]
		self.ydata=[]
		self.x = None
		self.y = None
		self.points = points

		# print self.points
		self.fig, self.ax = plt.subplots(1, 1)
		self.ax.axis('equal')
		self.ax.plot(self.points[:,0],self.points[:,1])
		if(bool(len(self.xdata))):
			self.ax.plot(self.xdata[-1],self.ydata[-1],'ro')
		self.lines, = self.ax.plot(self.xdata,self.ydata,color='g', linewidth=2.0)
		self.ax.set_autoscaley_on(True)
		self.ax.set_xlabel('X (m)')
		self.ax.set_ylabel('Y (m)')
		# self.ax.set_ylim([-40.0, 40.0])
		# self.ax.set_xlim([-40.0, 40.0])
		self.ax.grid()

	def PlotData(self, x, y):
		self.xdata.append(x)
		self.ydata.append(y)
		self.x = x
		self.y = y
		self.lines.set_data(self.xdata,self.ydata)
		self.ax.relim()
		self.ax.autoscale_view()
		#We need to draw *and* flush
		self.fig.canvas.draw()
		self.fig.canvas.flush_events()		
def main(argv):

	# namespace - 'carla'
	node = Navigation('carla')
	points = node.get_points()
	plt.plot(points[:,0],points[:,1])

	# plt.draw()
	# plt.pause(0.001)
	# print points
	controller_object = cn.purePursuit(points)
	plot = DynamicUpdate(points)
	r = rospy.Rate(50)

	while not rospy.is_shutdown():
		pos=rospy.wait_for_message("/carla/ego_vehicle/odometry",Odometry)

		x=pos.pose.pose.position.x
		y=pos.pose.pose.position.y
		plot.PlotData(x,y)
		controller_object.publish_()
		r.sleep()

if __name__ == '__main__':
	try:
		main(sys.argv[1:])
	except rospy.ROSInterruptException:
		pass