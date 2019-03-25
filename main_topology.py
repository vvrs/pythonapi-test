import carla
import time
import random 

import matplotlib.pyplot as plt
import numpy as np
import time

import networkx as nx
import math

from get_topology import *


def main():

	client = carla.Client('localhost',2000)
	client.set_timeout(2.0)

	world = client.get_world()

	blueprint_library = world.get_blueprint_library()


	# vehicle
	vehicle = blueprint_library.filter('vehicle.lin*')
	vehicle = vehicle[0]
	# get one of the possible spawning locations
	# transform = random.choice(world.get_map().get_spawn_points())
	# print transform
	# plt.show()
	


	# Get topology from the map
	_map = world.get_map()
	

	# Build waypoint graph
	topology,waypoints = get_topology(_map)
	# print topology[0].keys()#,type(topology[0])


	xs = waypoints[:,0]
	ys = waypoints[:,1]
	graph,id_map = build_graph(topology)
	e1 = graph.edges()[0]
	e1_data = graph.get_edge_data(e1[0],e1[1])

	source_location = e1_data['entry']
	waypoint_next_to_source = e1_data['path'][0]

	source_vector = e1_data['entry_vector']


	source_yaw = np.degrees(np.arctan2(source_vector[1],source_vector[0]))

	transform = carla.Transform(carla.Location(x=source_location[0], y=source_location[1], z=2), carla.Rotation(yaw=source_yaw))
	vehicle = world.spawn_actor(vehicle,transform)

	


	# print graph.get_edge_data()

	axes = plt.gca()
	axes.set_xlim(-500, 500)
	axes.set_ylim(-500, +500)
	line, = axes.plot(xs, ys, 'r*')

	p = get_shortest_path(graph, 0, 14)
	mapk = id_map.keys()
	srcind = id_map.values().index(0)
	print srcind
	destind = id_map.values().index(14)
	source = mapk[srcind]
	dest = mapk[destind]

	plt.plot(source[0],source[1],'go--', linewidth=2, markersize=12)
	plt.plot(dest[0],dest[1],'ro--', linewidth=2, markersize=12)

	# print len(final_path_x),len(final_path_y)
	plt.plot(p[:,0],p[:,1])
	plt.show()		



	
	# plt.show()

if __name__ == "__main__":
	main()