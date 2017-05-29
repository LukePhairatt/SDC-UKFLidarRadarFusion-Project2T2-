'''
@Punnu Phairatt 28/5/2017

This file reads the input log file for analysing the object trajectory with speeds and accelerations given by Lidar and Radar  
'''


import pandas as pd
from math import *
import numpy as np
import matplotlib.pyplot as plt


# read data
df = pd.DataFrame()
with open('obj_pose-laser-radar-synthetic-input.txt', 'r') as f:
    for line in f:
        df = pd.concat( [df, pd.DataFrame([tuple(line.strip().split('\t'))])], ignore_index=True)
        
# mapping lidar and radar data
n_rows = df.shape[0]
n_cols = df.shape[1]

data_radar = {'type':[],'rho':[],'phi':[],'rhodot':[],'times':[],'x_truth':[],'y_truth':[],'vx_truth':[],'vy_truth':[],'yaw_truth':[],'yawrate_truth':[]}
data_lidar = {'type':[],'px':[],'py':[],'times':[],'x_truth':[],'y_truth':[],'vx_truth':[],'vy_truth':[],'yaw_truth':[],'yawrate_truth':[]}
for i in range(1,n_rows):
    #reading row-by-row
    sensor_type = df.ix[i].values[0]
    if sensor_type == 'L':
        data_lidar['type'] = sensor_type
        data_lidar['px'].append(float(df.ix[i].values[1]))
        data_lidar['py'].append(float(df.ix[i].values[2]))
        data_lidar['times'].append(float(df.ix[i].values[3]))
        data_lidar['x_truth'].append(float(df.ix[i].values[4]))
        data_lidar['y_truth'].append(float(df.ix[i].values[5]))
        data_lidar['vx_truth'].append(float(df.ix[i].values[6]))
        data_lidar['vy_truth'].append(float(df.ix[i].values[7]))
        data_lidar['yaw_truth'].append(float(df.ix[i].values[8]))
        data_lidar['yawrate_truth'].append(float(df.ix[i].values[9])) 
    elif sensor_type == 'R':
        data_radar['type'] = sensor_type
        data_radar['rho'].append(float(df.ix[i].values[1]))
        data_radar['phi'].append(float(df.ix[i].values[2]))
        data_radar['rhodot'].append(float(df.ix[i].values[3]))
        data_radar['times'].append(float(df.ix[i].values[4]))
        data_radar['x_truth'].append(float(df.ix[i].values[5]))
        data_radar['y_truth'].append(float(df.ix[i].values[6]))
        data_radar['vx_truth'].append(float(df.ix[i].values[7]))
        data_radar['vy_truth'].append(float(df.ix[i].values[8]))
        data_radar['yaw_truth'].append(float(df.ix[i].values[9]))
        data_radar['yawrate_truth'].append(float(df.ix[i].values[10]))

        
# lidar #
print('##### Lidar #####')
# compute linear acceleration from the ground truth
times_l = (np.array(data_lidar['times']) - data_lidar['times'][0])/1e6    # second start from 0
dt_l = np.array([times_l[i+1]-times_l[i] for i in range(len(times_l)-1)])       # time step
v_l = np.sqrt( (np.array(data_lidar['vx_truth']))**2 + (np.array(data_lidar['vy_truth']))**2 ) 
a_l = np.array([(v_l[i+1]-v_l[i])/dt_l[i] for i in range(len(v_l)-1)])
print('mean abs(v): ',np.mean(abs(v_l)),',','std v', np.std(v_l))
print('mean abs(a): ',np.mean(abs(a_l)),',','std a', np.std(a_l))

# compute angular acceleration from the ground truth
theta_l = np.array(data_lidar['yaw_truth'])
w_l = np.array(data_lidar['yawrate_truth'])
wd_l = np.array([(w_l[i+1]-w_l[i])/dt_l[i] for i in range(len(w_l)-1)])
print('mean abs(yaw): ',np.mean(abs(theta_l)),',','std yaw', np.std(theta_l))
print('mean abs(yaw_v): ',np.mean(abs(w_l)),',','std yaw_v', np.std(w_l))
print('mean abs(yaw_w): ',np.mean(abs(wd_l)),',','std yaw_a', np.std(wd_l))

# radar #
print('##### Radar #####')
# compute linear acceleration from the ground truth
times_r = (np.array(data_radar['times']) - data_radar['times'][0])/1e6    # second start from 0
dt_r = np.array([times_r[i+1]-times_r[i] for i in range(len(times_r)-1)])       # time step
v_r = np.sqrt( (np.array(data_radar['vx_truth']))**2 + (np.array(data_radar['vy_truth']))**2 ) 
a_r = np.array([(v_r[i+1]-v_r[i])/dt_r[i] for i in range(len(v_r)-1)])
print('mean abs(v): ',np.mean(abs(v_r)),',','std v', np.std(v_r))
print('mean abs(a): ',np.mean(abs(a_r)),',','std a', np.std(a_r))

# compute angular acceleration from the ground truth
theta_r = np.array(data_radar['yaw_truth'])
w_r = np.array(data_radar['yawrate_truth'])
wd_r = np.array([(w_r[i+1]-w_r[i])/dt_r[i] for i in range(len(w_r)-1)])
print('mean abs(yaw): ',np.mean(abs(theta_r)),',','std yaw', np.std(theta_r))
print('mean abs(yaw_v): ',np.mean(abs(w_r)),',','std yaw_v', np.std(w_r))
print('mean abs(yaw_w): ',np.mean(abs(wd_r)),',','std wd', np.std(wd_r))

# plot
plt.figure(1)
plt.subplot(221)
plt.title('a-lidar')
plt.plot(times_l[1:],a_l)
plt.subplot(222)
plt.title('wd-lidar')
plt.plot(times_l[1:],wd_l)
plt.subplot(223)
plt.title('a-radar')
plt.plot(times_r[1:],a_r)
plt.subplot(224)
plt.title('wd-radar')
plt.plot(times_r[1:],wd_r)


# plot
plt.figure(2)
plt.subplot(231)
plt.title('v-lidar')
plt.plot(times_l,v_l)
plt.subplot(232)
plt.title('yaw-lidar')
plt.plot(times_l,theta_l)
plt.subplot(233)
plt.title('yaw-rate-lidar')
plt.plot(times_l,w_l)
plt.subplot(234)
plt.title('v-rsdar')
plt.plot(times_r,v_r)
plt.subplot(235)
plt.title('yaw-rsdar')
plt.plot(times_r,theta_r)
plt.subplot(236)
plt.title('yaw-rate-radar')
plt.plot(times_r,w_r)

plt.show()


'''
result:
##### Lidar #####
mean abs(v):  5.00000000681 , std v 0.141421352371
mean abs(a):  0.0642469843708 , std a 0.0712212269419
mean abs(yaw):  2.18835166374 , std yaw 1.54742884593
mean abs(yaw_v):  0.350150084512 , std yaw_v 0.388908727097
mean abs(yaw_w):  0.0877982873896 , std yaw_a 0.0975428076523
##### Radar #####
mean abs(v):  5.00000000546 , std v 0.141421368767
mean abs(a):  0.0642367457639 , std a 0.0712210544612
mean abs(yaw):  2.18835166374 , std yaw 1.54742884593
mean abs(yaw_v):  0.35012244848 , std yaw_v 0.388908735731
mean abs(yaw_w):  0.0877913594378 , std wd 0.0975428723436
'''


        
        
        
        
        
        
        
        

