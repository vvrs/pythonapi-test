import carla
import time
import random 

import matplotlib.pyplot as plt
import numpy as np
import time

import networkx as nx
import math
# plt.ion()
global debug
debug = 0
def unit_vector(point1, point2):

	x1, y1, _ = point1
	x2, y2, _ = point2

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
		z1 = segment[0].transform.location.z

		x2 = segment[1].transform.location.x
		y2 = segment[1].transform.location.y
		z2 = segment[1].transform.location.z

		seg_dict = dict()
		seg_dict['entry'] = (x1, y1, z1)
		seg_dict['exit'] = (x2, y2, z2)
		seg_dict['path'] = []
		wp1 = segment[0]
		wp2 = segment[1]
		seg_dict['intersection'] = True if wp1.is_intersection else False
		endloc = wp2.transform.location
		w = wp1.next(1)[0]
		while w.transform.location.distance(endloc) > 1:
			x = w.transform.location.x
			y = w.transform.location.y
			z = w.transform.location.z
			seg_dict['path'].append((x, y, z))
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
	final_path_z = []
	for i in range(len(p)-1):
		edge_ =  graph.get_edge_data(p[i],p[i+1])
		# waypoint_ = edge_['entry']
		path_ = edge_['path']
		path_ = np.array(path_)

		if debug:
			print __file__, "path ",path_

		try:
			xs = path_[:,0]
			ys = path_[:,1]
			zs = path_[:,2]
			final_path_x.extend(list(xs))
			final_path_y.extend(list(ys))
			final_path_z.extend(list(zs))
		except IndexError:
			pass
			
	final_path_x = np.array(final_path_x).reshape(len(final_path_x),1)
	final_path_y = np.array(final_path_y).reshape(len(final_path_y),1)
	final_path_z = np.array(final_path_z).reshape(len(final_path_z),1)

	return np.hstack((final_path_x, final_path_y, final_path_z))

