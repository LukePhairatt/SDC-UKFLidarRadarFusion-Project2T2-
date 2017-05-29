'''
@Punnu Phairatt 28/5/2017

This file reads the output log file for the XY path and NIS   
'''

import pandas as pd
from math import *
import numpy as np
import matplotlib.pyplot as plt


# read data
df = pd.DataFrame()
with open('obj_pose-laser-radar-synthetic-output.txt', 'r') as f:
    for line in f:
        df = pd.concat( [df, pd.DataFrame([tuple(line.strip().split('\t'))])], ignore_index=True)
        
# get header name
col_name = df[0:1].values[0]
# remove header name
df = df[1:]
        
# mapping lidar and radar data
n_rows = df.shape[0]
n_cols = df.shape[1]


data_out = {'time_stamp':[],'px_state':[],'py_state':[],'v_state':[],
            'yaw_angle_state':[],'yaw_rate_state':[],'sensor_type':[],
            'NIS':[],'px_measured':[],'py_measured':[],'px_ground_truth':[],
            'py_ground_truth':[],'vx_ground_truth':[],'vy_ground_truth':[]}

for i in range(1,n_rows):
    #reading row-by-row
    if not (df.ix[i].values[0] == ''):
        data_out['time_stamp'].append(float(df.ix[i].values[0]))
        data_out['px_state'].append(float(df.ix[i].values[1]))
        data_out['py_state'].append(float(df.ix[i].values[2]))
        data_out['v_state'].append(float(df.ix[i].values[3]))
        data_out['yaw_angle_state'].append(float(df.ix[i].values[4]))
        data_out['yaw_rate_state'].append(float(df.ix[i].values[5]))
        data_out['sensor_type'].append(df.ix[i].values[6])
        data_out['NIS'].append(float(df.ix[i].values[7]))
        data_out['px_measured'].append(float(df.ix[i].values[8]))
        data_out['py_measured'].append(float(df.ix[i].values[9]))
        data_out['px_ground_truth'].append(float(df.ix[i].values[10]))
        data_out['py_ground_truth'].append(float(df.ix[i].values[11]))
        data_out['vx_ground_truth'].append(float(df.ix[i].values[12]))
        data_out['vy_ground_truth'].append(float(df.ix[i].values[13]))

        
# xy path
times = (np.array(data_out['time_stamp']) - data_out['time_stamp'][0])/1e6    # second start from 0
dt = np.array([times[i+1]-times[i] for i in range(len(times)-1)])       # time step
px = np.array(data_out['px_state'])
py = np.array(data_out['py_state'])
px_gt = np.array(data_out['px_ground_truth'])
py_gt = np.array(data_out['py_ground_truth'])

# nis
nis =  np.array(data_out['NIS'])
nis_target = 7.8*np.ones(len(nis))

# plot xy
plt.figure(1)
plt.title('Object XY-Path')
plt.plot(px_gt,py_gt,'r.',linewidth=5,label='ground truth')
plt.plot(px,py,linewidth=2,label='UKF')
plt.legend(loc='upper left')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.axis('equal')
#plt.grid()
plt.text(10, -8, 'RMSE\n'+ \
                  'X: 0.0695387\n' + \
                  'Y: 0.0894829\n' + \
                  'Vx:0.386345\n' + \
                  'Vy:0.171645', fontsize = 10, color = 'k')

# plot NIS
plt.figure(2)
plt.title('Normalised Innovation Squared Radar and Lidar')
plt.plot(nis_target,'b',label='7.8 NIS target')
plt.plot(nis,'r',linewidth=2)
plt.legend(loc='upper left')
plt.xlabel('N-samples')
plt.ylabel('Innovation')
plt.show()





