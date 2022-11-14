import numpy as np
import modern_robotics as mr
from traj_gen import TrajectoryGenerator

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