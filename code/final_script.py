import numpy as np
import modern_robotics as mr
from traj_gen import TrajectoryGenerator
from find_next_state import NextState
from control import FeedbackControl
import matplotlib.pyplot as plt

# Generate Trajectory
Tse_initial = np.array([[0,0,1,0],
                        [0,1,0,0],
                        [-1,0,0,0.5],
                        [0,0,0,1]])

Tsc_initial = np.array([[1,0,0,1],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]])

Tsc_final = np.array([[0,1,0,0],
                        [-1,0,0,-1],
                        [0,0,1, 0],
                        [0,0,0,1]])

rot = np.sqrt(2.0)/2.0

Tce_grasp = np.array([[-rot,0,rot,0],
                        [0,1,0,0],
                        [-rot,0,-rot,0],
                        [0,0,0,1]])

Tce_standoff = np.array([[-rot,0,rot,0],
                            [0,1,0,0],
                            [-rot,0,-rot,0.2],
                            [0,0,0,1]])

final_trajectory = TrajectoryGenerator(Tse_initial,Tsc_initial,Tsc_final,Tce_grasp,Tce_standoff,k=1)

# define initial configuration
config_i = np.array([np.pi/10,-0.5,0.5,np.pi/10,-np.pi/10,-np.pi/10,-np.pi/10,0,0,0,0,0,0]).reshape(1,13)

# adjust Kp and Ki here!!
Kp = 2.2
Ki = 0


# here we loop
dt = 0.01
config_list = []
error_list = []

config_list.append(config_i)

for i in np.arange(np.shape(final_trajectory)[0]-1):
    # here we write down Xd
    Xd1 = np.array([[final_trajectory[i,0],final_trajectory[i,1],final_trajectory[i,2],final_trajectory[i,9]],
                    [final_trajectory[i,3],final_trajectory[i,4],final_trajectory[i,5],final_trajectory[i,10]],
                    [final_trajectory[i,6],final_trajectory[i,7],final_trajectory[i,8],final_trajectory[i,11]],
                    [0,0,0,1]])

    # here we write down Xd_next
    Xd2 = np.array([[final_trajectory[i+1,0],final_trajectory[i+1,1],final_trajectory[i+1,2],final_trajectory[i+1,9]],
                    [final_trajectory[i+1,3],final_trajectory[i+1,4],final_trajectory[i+1,5],final_trajectory[i+1,10]],
                    [final_trajectory[i+1,6],final_trajectory[i+1,7],final_trajectory[i+1,8],final_trajectory[i+1,11]],
                    [0,0,0,1]])

    # use FeedbackControl to get V
    V, commanded_speed, error = FeedbackControl(config_i,Xd1,Xd2,Kp,Ki,dt)

    # here we adjust the order of wheels speeds and joints speeds
    commanded_speed = np.array([commanded_speed[4],
                                commanded_speed[5],
                                commanded_speed[6],
                                commanded_speed[7],
                                commanded_speed[8],
                                commanded_speed[0],
                                commanded_speed[1],
                                commanded_speed[2],
                                commanded_speed[3]]).reshape(1,9)
    max_speed = 1000

    # here we compute new configuration
    config_i = NextState(config_i,commanded_speed,dt,max_speed)
    # gripper state
    config_i = np.append(config_i,[[final_trajectory[i,12]]],axis=1)
    config_list.append(config_i)
    error_list.append(error)

config_arr = np.squeeze(config_list)
error_list = np.squeeze(error_list)

np.savetxt("final.csv", config_arr, delimiter=',')
"""
f = open("~/ME449/Mobile_Manipulation/final.csv", "w") 
for i in range(np.shape(config_arr)[0]):
    output = "%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f\n" % (config_arr[i,0],
                                                                                                               config_arr[i,1],
                                                                                                               config_arr[i,2],
                                                                                                               config_arr[i,3],
                                                                                                               config_arr[i,4],
                                                                                                               config_arr[i,5],
                                                                                                               config_arr[i,6],
                                                                                                               config_arr[i,7],
                                                                                                               config_arr[i,8],
                                                                                                               config_arr[i,9],
                                                                                                               config_arr[i,10],
                                                                                                               config_arr[i,11],
                                                                                                               config_arr[i,12])
    f.write(output)
f.close()
"""

# plot the error
steps = np.arange(2800)
plt.figure(dpi=110,facecolor='w')
plt.plot(steps,error_list[:,0])
plt.plot(steps,error_list[:,1])
plt.plot(steps,error_list[:,2])
plt.plot(steps,error_list[:,3])
plt.plot(steps,error_list[:,4])
plt.plot(steps,error_list[:,5])
plt.title("Error plot")
plt.xlabel('Time steps')
plt.ylabel('Error')
plt.legend([r'$w_{e1}$',r'$w_{e2}$',r'$w_{e3}$',r'$\dot{p}_{e1}$',r'$\dot{p}_{e2}$',r'$\dot{p}_{e3}$'])
plt.grid(True)
plt.show()