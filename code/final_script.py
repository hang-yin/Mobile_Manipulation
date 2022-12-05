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

# Original Task

Tsc_initial = np.array([[1,0,0,1],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]])

Tsc_final = np.array([[0,1,0,0],
                      [-1,0,0,-1],
                      [0,0,1, 0],
                      [0,0,0,1]])
"""
# New Task

Tsc_initial = np.array([[1,0,0,1],
                        [0,1,0,-1],
                        [0,0,1,0],
                        [0,0,0,1]])

Tsc_final = np.array([[0,1,0,0],
                      [-1,0,0,0],
                      [0,0,1, 0],
                      [0,0,0,1]])
"""

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
config_i = np.array([0.8,
                     -0.4,
                     0.25,
                     0.133,
                     -0.338,
                     -0.474,
                     -0.674,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0.0,
                     0]).reshape(1,13)

# adjust Kp and Ki here for "best" or "overshoot" conditions
# best: Kp = 2.5, Ki = 0
Kp = 2.5
Ki = 0
# overshoot: Kp = 2.0, Ki = 50.0
# Kp = 2.0
# Ki = 50.0

# here we loop
dt = 0.01
config_list = []
error_list = []

config_list.append(config_i)

print("Computing trajectory...\n")
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
    max_speed = 10000

    # here we compute new configuration
    config_i = NextState(config_i,commanded_speed,dt,max_speed)
    # gripper state
    config_i = np.append(config_i,[[final_trajectory[i,12]]],axis=1)
    config_list.append(config_i)
    error_list.append(error)

config_arr = np.squeeze(config_list)
error_list = np.squeeze(error_list)


print("Saving data to final.csv\n")
np.savetxt("final.csv", config_arr, delimiter=',')

print("Saving error data to error.csv\n")
np.savetxt("error.csv", error_list, delimiter=',')

# plot the error
print("Plotting error")
steps = np.arange(2800)
plt.figure(dpi=100)
plt.title("Error plot")
plt.xlabel('Time steps')
plt.ylabel('Error')
plt.plot(steps,error_list[:,0])
plt.plot(steps,error_list[:,1])
plt.plot(steps,error_list[:,2])
plt.plot(steps,error_list[:,3])
plt.plot(steps,error_list[:,4])
plt.plot(steps,error_list[:,5])
plt.legend(['Xerr0','Xerr1','Xerr2','Xerr3','Xerr4','Xerr5'])
plt.show()