#!/usr/bin/env python
# -*- coding: utf-8 -*-

# third party packages
import matplotlib.pyplot as plt
import numpy as np

# local packages
from optscan import get_path_statistics, create_scan_grid, find_good_path

# initializing parameters
params = dict()

#### Edit session ####
params['hrange'] = 8 # in microns
params['vrange'] = 8 # in microns
params['step_size'] = 2 # in microns
params['central_point'] = True
params['center']  = (0,0)
params['scangrid'] = 'ROUND_ROI'
params['expo_time'] = 0.5 # in seconds
params['overhead_time'] = 0.25 # in seconds
######################

## Before optimization
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

## Optimization
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
multiplier = 1 # in case of change of units
# display figures
# before optimization
fig1 = plt.figure(1)
plt.clf()
ax1 = fig1.add_subplot(111)
im1 = ax1.plot(pl[:,0]*multiplier,pl[:,1]*multiplier,'ro-')
ax1.plot(pl[:,0][0]*multiplier,pl[:,1][0]*multiplier,'b^-')
ax1.plot(pl[:,0][-1]*multiplier,pl[:,1][-1]*multiplier,'gs-')
ax1.set_xlabel('Horizontal motor positions ($\mu$m)',fontsize = 18)
ax1.set_ylabel('Vertical motor positions ($\mu$m)',fontsize = 18)
ax1.set_title('Motor positions (Before optimization)')
ax1.tick_params(labelsize = 16)
plt.axis('image')
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
ax2.set_xlabel('Horizontal motor positions ($\mu$m)',fontsize = 18)
ax2.set_ylabel('Vertical motor positions ($\mu$m)',fontsize = 18)
ax2.set_title('Motor positions (After optimization - Euclidean)')
ax2.tick_params(labelsize = 16)
plt.axis('image')
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
ax3.set_xlabel('Horizontal motor positions ($\mu$m)',fontsize = 18)
ax3.set_ylabel('Vertical motor positions ($\mu$m)',fontsize = 18)
ax3.set_title('Motor positions (After optimization - Chebyshev)')
ax3.tick_params(labelsize = 16)
plt.axis('image')
plt.tight_layout()
plt.show(block=False)
plt.savefig('grid_optimized_Chebyshev_{}x{}_{}.png'.format(params['hrange'],
            params['vrange'],params['step_size']),bbox_inches='tight',dpi=200)
