import carla
import time
import random 

import matplotlib.pyplot as plt
import numpy as np
import time

import networkx as nx
import math
# plt.ion()

def unit_vector(point1, point2):

	x1, y1 = point1
	x2, y2 = point2

	vector = (x2 - x1, y2 - y1)
	vector_mag = math.sqrt(vector[0]**2 + vector[1]**2)
	vector = (vector[0] / vector_mag, vector[1] / vector_mag)

	return vector
def get_topology(map):
	_topology = map.get_topology()
	topology = []
	xs = []
	ys = []
	zs = []

	for segment in _topology:
		x1 = segment[0].transform.location.x
		y1 = segment[0].transform.location.y
		x2 = segment[1].transform.location.x
		y2 = segment[1].transform.location.y
		seg_dict = dict()
		seg_dict['entry'] = (x1, y1)
		seg_dict['exit'] = (x2, y2)
		seg_dict['path'] = []
		wp1 = segment[0]
		wp2 = segment[1]
		seg_dict['intersection'] = True if wp1.is_intersection else False
		endloc = wp2.transform.location
		w = wp1.next(1)[0]
		while w.transform.location.distance(endloc) > 1:
			x = w.transform.location.x
			y = w.transform.location.y
			seg_dict['path'].append((x, y))
			w = w.next(1)[0]

		topology.append(seg_dict)

		for j in segment:
			x = j.transform.location.x
			y = j.transform.location.y
			z = j.transform.location.z
			xs.append(x)
			ys.append(y)
			zs.append(z)
	xs = np.array(xs).reshape((len(xs),1))
	ys = np.array(ys).reshape((len(ys),1))
	zs = np.array(zs).reshape((len(zs),1))


	waypoints = np.hstack((xs,ys,zs))

	return topology,waypoints

def build_graph(topology):
	graph = nx.DiGraph()
	# Map with structure {(x,y): id, ... }
	id_map = dict()

	for segment in topology:

		entryxy = segment['entry']
		exitxy = segment['exit']
		path = segment['path']
		intersection = segment['intersection']
		for vertex in entryxy, exitxy:
			# print('inside build graph...')
			# Adding unique nodes and populating id_map
			if vertex not in id_map:
				new_id = len(id_map)
				id_map[vertex] = new_id
				graph.add_node(new_id, vertex=vertex)

		n1, n2 = id_map[entryxy], id_map[exitxy]
		# print n1,n2
		# Adding edge with attributes
		graph.add_edge(
			n1, n2,
			length=len(path) + 1, path=path,
			entry = entryxy,
			exit = exitxy,
			entry_vector=unit_vector(
				entryxy, path[0] if len(path) > 0 else exitxy),
			exit_vector=unit_vector(
				path[-1] if len(path) > 0 else entryxy, exitxy),
			net_vector=unit_vector(entryxy, exitxy),
			intersection=intersection)
	return graph,id_map

def get_shortest_path(graph, source, destination):
	p = nx.shortest_path(graph,source=source,target=destination)
	final_path_x = []
	final_path_y = []
	for i in range(len(p)-1):
		edge_ =  graph.get_edge_data(p[i],p[i+1])
		# waypoint_ = edge_['entry']
		path_ = edge_['path']
		path_ = np.array(path_)
		try:
			xs = path_[:,0]
			ys = path_[:,1]
			final_path_x.extend(list(xs))
			final_path_y.extend(list(ys))
		except IndexError:
			pass
			
	final_path_x = np.array(final_path_x).reshape(len(final_path_x),1)
	final_path_y = np.array(final_path_y).reshape(len(final_path_y),1)
	return np.hstack((final_path_x,final_path_y))

def main():

	client = carla.Client('localhost',2000)
	client.set_timeout(2.0)

	world = client.get_world()

	blueprint_library = world.get_blueprint_library()


	# vehicle
	vehicle = blueprint_library.filter('vehicle.lin*')

	# get one of the possible spawning locations
	transform = random.choice(world.get_map().get_spawn_points())
	print transform
	# plt.show()
	


	# Get topology from the map
	_map = world.get_map()
	

	# Build waypoint graph
	topology,waypoints = get_topology(_map)
	print topology[0].keys()#,type(topology[0])


	xs = waypoints[:,0]
	ys = waypoints[:,1]
	graph,id_map = build_graph(topology)

	graph

	axes = plt.gca()
	axes.set_xlim(-500, 500)
	axes.set_ylim(-500, +500)
	line, = axes.plot(xs, ys, 'r*')

	p = get_shortest_path(graph, 0, 14)
	mapk = id_map.keys()
	srcind = id_map.values().index(0)
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
