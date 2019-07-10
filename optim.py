#!/usr/bin/env python
# -*- coding: utf-8 -*-

# standard packages
from collections import namedtuple
import math
import time

# third party packages
import matplotlib.pyplot as plt
import numpy as np
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
from scipy.spatial import distance

PathStats = namedtuple('PathStats', ['x_len', 'y_len', 'p_tim'])

def get_path_statistics(path):
    """
    <path> : a list (or numpy array) of points
    Returns a namedtuple of infos about <path> : (hrz_length, vrt_length)
    """
    x_speed = 4.0
    y_speed = 1.0
    
    x_dists = [abs(j[0]-i[0]) for i, j in list(zip(path[:-1], path[1:]))]
    y_dists = [abs(j[1]-i[1]) for i, j in list(zip(path[:-1], path[1:]))]

    x_length = sum(x_dists)
    y_length = sum(y_dists)
    
    p_time = sum(np.maximum(np.array(x_dists) / x_speed, np.array(y_dists) / y_speed))

    return PathStats(x_length, y_length, p_time)

def round_roi(hrz_range, vrt_range, step_size, n_1shell = 5, cenh = 0, cenv = 0, central_point=True):
    """
    Returns a numpy array of points.
    """

    print("Doing Round_roi calculation with parameteres:")
    print("hrz_range={}, vrt_range={}, step_size={}".format(hrz_range, vrt_range, step_size))

    lv = vrt_range # height of the ROI
    lh = hrz_range # width of the ROI
    dr = step_size

    rmax = np.sqrt((lv/2)**2+(lh/2)**2) # maximum size (this makes the difference between roundscans and roundroi)
    nr = int(1 + np.floor(rmax/dr))
    nth = n_1shell # number of points in the first shell
    pos_real = []

    angle_shift = 0 # angular rotation

    for ir in range(nr+1):
        rr = ir*dr # defines the incremental radial step
        if ir == 0:
            if central_point:
                pos_real.append([0.0,0.0])
        else:
            dth = 2*np.pi/(nth*ir) # defines the angular step in each shell
        for ith in range(nth*ir):
            th = ith*dth + angle_shift # angular points in the shell
            xy = rr * np.array([np.sin(th),np.cos(th)]) # positions of the points
            if ((np.abs(xy[0]) >= lh/2) or (np.abs(xy[1]) >= lv/2)):
                continue
            pos_real.append(xy)
    pos_real = np.array(pos_real)
    # absolute center positions
    pos_real[:,0] = pos_real[:,0] + cenv # recenter vertical
    pos_real[:,1] = pos_real[:,1] + cenh # recenter horizontal
    numpts = pos_real.shape[0]
    print('The scan grid contains {} points'.format(numpts))
    
    return pos_real

def calculate_distribution(hrange, vrange, step_size, n_1shell = 5, cenh = 0, cenv = 0, central_point = True):
    # point_list is a numpy array of points
    point_list = round_roi(hrange, vrange, step_size, n_1shell, cenh, cenv, central_point)

    array1, array2 = list(zip(*point_list))

    # Calculates number of points.
    point_count = len(array1)
    
    return point_list#, point_count, path_len_horizontal, path_len_vertical, path_time

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

if __name__ == '__main__':
    # initializing parameters
    params = dict()
    #### edit session ####
    params['hrange'] = 40 # in microns
    params['vrange'] = 20 # in microns
    params['step_size'] = 2 # in microns
    params['central_point'] = True 
    params['center']  = (0,0)
    params['scangrid'] = 'ROUND_ROI'
    params['expo_time'] = 0.5 # in seconds
    params['overhead_time'] = 0.25 # in seconds
    ######################
    # before optimization
    print('Creating scan grid')
    datai = create_scan_grid(**params)
    pl = datai['locations']
    pc = len(pl)
    # Calculates Statistics : paths lengths and time
    ph, pv, pt = get_path_statistics(pl)
    Tt = pt + pc*(params['expo_time'] + params['overhead_time']) # total time
    # print results
    # before
    print('\n#---------------------------------#')
    print('#      Before optimization        #')
    print('#---------------------------------#')
    print('Point counts: {:d}'.format(pc))
    print('Path horizontal length: {:.03f} microns'.format(ph))
    print('Path Vertical length: {:.03f} microns'.format(pv))
    print('Path total time: {:.02f} s'.format(pt))
    print('Total time: {:.02f} s'.format(Tt))
    print('#---------------------------------#\n')
    
    print('Starting optimization')
    # Euclidean distance
    print('Using Euclidean distance')
    params['distance_method'] = 'Euclidean'
    datae = find_good_path(datai,params['distance_method'])#main(**params)
    plopte = datae['trajectory']
    pcopte = len(plopte)
    # Calculates Statistics : paths lengths and time
    phopte, pvopte, ptopte = get_path_statistics(plopte)
    Ttopte = ptopte + pcopte*(params['expo_time'] + params['overhead_time'])
    # after optimization (Euclidean)
    print('#---------------------------------#')
    print('# After optimization - Euclidean  #')
    print('#---------------------------------#')
    print('Point counts: {:d}'.format(pcopte))
    print('Path horizontal length: {:.03f} microns'.format(phopte))
    print('Path Vertical length: {:.03f} microns'.format(pvopte))
    print('Path total time: {:.02f} s'.format(ptopte))
    print('Total time: {:.02f} s'.format(Ttopte))
    print('#---------------------------------#\n')
    
    # Chebyshev distance
    print('Using Chebyshev distance')
    params['distance_method'] = 'Chebyshev'
    datac = find_good_path(datai,params['distance_method'])#main(**params)
    ploptc = datac['trajectory']
    pcoptc = len(ploptc)
    # Calculates Statistics : paths lengths and time
    phoptc, pvoptc, ptoptc = get_path_statistics(ploptc)
    Ttoptc = ptoptc + pcoptc*(params['expo_time'] + params['overhead_time'])
    # after optimization (Chebyshev)
    print('#---------------------------------#')
    print('# After optimization - Chebyshev  #')
    print('#---------------------------------#')
    print('Point counts: {:d}'.format(pcoptc))
    print('Path horizontal length: {:.03f} microns'.format(phoptc))
    print('Path Vertical length: {:.03f} microns'.format(pvoptc))
    print('Path total time: {:.02f} s'.format(ptoptc))
    print('Total time: {:.02f} s'.format(Ttoptc))
    print('#---------------------------------#\n')

    # display plots
    plt.close('all')
    multiplier = 1
    # display figures
    # before optimization
    fig1 = plt.figure(1)
    plt.clf()
    ax1 = fig1.add_subplot(111)
    im1 = ax1.plot(pl[:,0]*multiplier,pl[:,1]*multiplier,'ro-')
    ax1.plot(pl[:,0][0]*multiplier,pl[:,1][0]*multiplier,'b^-')
    ax1.plot(pl[:,0][-1]*multiplier,pl[:,1][-1]*multiplier,'gs-')
    #~ ax1.set_title('Before optimization')
    ax1.set_xlabel('Horizontal motor positions ($\mu$m)',fontsize = 18)
    ax1.set_ylabel('Vertical motor positions ($\mu$m)',fontsize = 18)
    ax1.set_title('Motor positions (Before optimization)')
    ax1.tick_params(labelsize = 16)
    plt.axis('image')
    #~ plt.axis('tight')
    plt.tight_layout()
    plt.show(block=False)
    plt.savefig('grid_calculated_nooptim_{}x{}_{}.png'.format(params['hrange'],
                params['vrange'],params['step_size']),bbox_inches='tight',dpi=200)
    
    # After optimization (Euclidean)
    fig2 = plt.figure(2)
    plt.clf()
    ax2 = fig2.add_subplot(111)
    im2 = ax2.plot(plopte[:,0]*multiplier,plopte[:,1]*multiplier,'ro-')
    ax2.plot(plopte[:,0][0]*multiplier,plopte[:,1][0]*multiplier,'b^-')
    ax2.plot(plopte[:,0][-1]*multiplier,plopte[:,1][-1]*multiplier,'gs-')
    #~ ax1.set_title('Before optimization')
    ax2.set_xlabel('Horizontal motor positions ($\mu$m)',fontsize = 18)
    ax2.set_ylabel('Vertical motor positions ($\mu$m)',fontsize = 18)
    ax2.set_title('Motor positions (After optimization - Euclidean)')
    ax2.tick_params(labelsize = 16)
    plt.axis('image')
    #~ plt.axis('tight')
    plt.tight_layout()
    plt.show(block=False)
    plt.savefig('grid_optimized_Euclidean_{}x{}_{}.png'.format(params['hrange'],
                params['vrange'],params['step_size']),bbox_inches='tight',dpi=200)
    
    # After optimization (Chebyshev)
    fig3 = plt.figure(3)
    plt.clf()
    ax3 = fig3.add_subplot(111)
    im3 = ax3.plot(ploptc[:,0]*multiplier,ploptc[:,1]*multiplier,'ro-')
    ax3.plot(ploptc[:,0][0]*multiplier,ploptc[:,1][0]*multiplier,'b^-')
    ax3.plot(ploptc[:,0][-1]*multiplier,ploptc[:,1][-1]*multiplier,'gs-')
    #~ ax1.set_title('Before optimization')
    ax3.set_xlabel('Horizontal motor positions ($\mu$m)',fontsize = 18)
    ax3.set_ylabel('Vertical motor positions ($\mu$m)',fontsize = 18)
    ax3.set_title('Motor positions (After optimization - Chebyshev)')
    ax3.tick_params(labelsize = 16)
    plt.axis('image')
    #~ plt.axis('tight')
    plt.tight_layout()
    plt.show(block=False)
    plt.savefig('grid_optimized_Chebyshev_{}x{}_{}.png'.format(params['hrange'],
                params['vrange'],params['step_size']),bbox_inches='tight',dpi=200)
