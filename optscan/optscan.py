#!/usr/bin/env python
# -*- coding: utf-8 -*-

# standard packages
from collections import namedtuple
import math
import time

# third party packages
import numpy as np
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

# local packages
from .scan_grids import calculate_distribution

__all__=['get_path_statistics',
         'create_scan_grid',
         'compute_distance_matrix',
         'print_solution',
         'find_good_path']

PathStats = namedtuple('PathStats', ['x_len', 'y_len', 'p_tim'])

def get_path_statistics(path, x_speed, y_speed):
    """
    <path> : a list (or numpy array) of points
    Returns a namedtuple of infos about <path> : (hrz_length, vrt_length)
    """
    #x_speed = 4.0
    #y_speed = 1.0

    x_dists = [abs(j[0]-i[0]) for i, j in list(zip(path[:-1], path[1:]))]
    y_dists = [abs(j[1]-i[1]) for i, j in list(zip(path[:-1], path[1:]))]

    x_length = sum(x_dists)
    y_length = sum(y_dists)

    p_time = sum(np.maximum(np.array(x_dists) / x_speed, np.array(y_dists) / y_speed))

    return PathStats(x_length, y_length, p_time)

def create_scan_grid(**params):
    """
    Creates the scanning grid and stores the data in a dictionary
    """
    hrange = params['hrange']
    vrange = params['vrange']
    step_size = params['step_size']
    n_1shell = 5 # default value
    cenh = params['center'][0]
    cenv = params['center'][1]
    central_point = params['central_point']
    # initializing the data dictionary
    data = {}
    # Locations in block units
    data['locations'] = calculate_distribution(hrange, vrange, step_size, n_1shell, cenh, cenv, central_point)
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data

def compute_distance_matrix(locations, distance_method):
    """
    Creates callback to return distance between points
    """
    distances = {}
    distance_factor = 4 # strecth in one direction because of different of speed of motors
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                if distance_method == 'Euclidean':
                    distances[from_counter][to_counter] = (int(
                        math.hypot((from_node[0] - to_node[0]),
                               distance_factor*(from_node[1] - to_node[1]))))
                elif distance_method == 'Chebyshev':
                    distances[from_counter][to_counter] = (int(
                        max(np.abs(from_node[0] - to_node[0]),
                               distance_factor*np.abs(from_node[1] - to_node[1]))))
                else:
                    raise ValueError('Unsupported distance method')
    return distances

def print_solution(manager, routing, assignment):
    """
    Prints assignment on console and return point index list
    """
    point_index_list = [0]
    #print('Objective: {}'.format(assignment.ObjectiveValue()))
    index = routing.Start(0)
    plan_output = 'Route:\n'
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        previous_index = index
        index = assignment.Value(routing.NextVar(index))
        point_index_list.append(index)
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    #print(plan_output)
    plan_output += 'Objective: {}m\n'.format(route_distance)

    return point_index_list

def find_good_path(data, distance_method = "Euclidean"):
    """
    Optimize the scan trajectory and returns a list of 2 lists:
    [ [Xcoords],  [Ycoords]]
    Derived from:
    https://developers.google.com/optimization/routing/tsp
    Github:
    https://github.com/google/or-tools/blob/stable/ortools/constraint_solver/samples/tsp_cities.py
    """
    # Create routing model
    if len(data['locations']) > 0:
        t0 = time.time()
        # Create the routing index manager
        manager = pywrapcp.RoutingIndexManager(
            len(data['locations']), data['num_vehicles'], data['depot'])

        # Create Routing Model
        routing = pywrapcp.RoutingModel(manager)

        # Create the distance matrix
        distance_matrix = compute_distance_matrix(data['locations'],distance_method)

        # Create the distance callback, which takes two arguments (the from and to node indices)
        # and returns the distance between these nodes.
        def distance_callback(from_index, to_index):
            """Returns the distance between the two nodes."""
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return distance_matrix[from_node][to_node]

        # transit callback
        transit_callback_index = routing.RegisterTransitCallback(distance_callback)

        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Setting first solution heuristic: the
        # method for finding a first solution to the problem.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

        # Solve the problem, return a solution if any.
        assignment = routing.SolveWithParameters(search_parameters)

        if assignment:
            point_index_list = print_solution(manager, routing, assignment)
            point_index_list.pop()
            print('Done. Time elapsed: {:.02f} s'.format(time.time()-t0))
            # updating data dictionary
            data['trajectory'] = np.array(
            [[data['locations'][idx][0],data['locations'][idx][1]] for idx in point_index_list])
            return data
        else:
            print('No solution found.')
    else:
        print('Specify an instance greater than 0.')
