#!/usr/bin/env python
# -*- coding: utf-8 -*-

# third party packages
import matplotlib.pyplot as plt
import numpy as np

__all__=['round_roi',
         'calculate_distribution']

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
