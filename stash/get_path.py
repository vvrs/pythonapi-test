import sys
try:
	sys.path.append('../PythonAPI')
except IndexError:
	pass

import random

import carla
from agents.navigation.agent import *
from agents.navigation.local_planner import LocalPlanner
from agents.navigation.local_planner import compute_connection, RoadOption
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.tools.misc import vector

def main():

	client = carla.Client('localhost',2000)
	client.set_timeout(2.0)

	world = client.get_world()
	_map = world.get_map()
	dao = GlobalRoutePlannerDAO(_map)

	grp = GlobalRoutePlanner(dao)
	grp.setup()

	blueprint_library = world.get_blueprint_library()

	# vehicle
	blueprint = random.choice(world.get_blueprint_library().filter('vehicle.lin*'))

	spawn_points = world.get_map().get_spawn_points()
	spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()

	_vehicle = world.try_spawn_actor(blueprint, spawn_point)
	print(_vehicle)
	start_waypoint = _map.get_waypoint(_vehicle.get_location())
	end_waypoint = random.choice(spawn_points) if spawn_points else carla.Transform()

	# Obtain route plan
	x1 = start_waypoint.transform.location.x
	y1 = start_waypoint.transform.location.y
	x2 = end_waypoint.location.x
	y2 = end_waypoint.location.y
	print(x1,x2,y1,y2)
	_graph,_id_map = grp.build_graph()
	print(_graph)

if __name__ == '__main__':
	main()