import numpy as np
import modern_robotics as mr

def TrajectoryGenerator(Tse_initial,Tsc_initial,Tsc_final,Tce_grasp,Tce_standoff,k):

    time_step = 0.01
    ##################################
    # segment 1: A trajectory to move the gripper from its initial configuration to a "standoff" configuration a few cm above the block
    # represent the frame of end-effector when standoff in space frame
    Tse_standoff_before = Tsc_initial.dot(Tce_standoff)
    segment1 = mr.CartesianTrajectory(Tse_initial,Tse_standoff_before,5.0,5.0/time_step,3)
    
    ##################################
    # segment 2: A trajectory to move the gripper down to the grasp position
    # represent the frame of end-effector in space frame in segment 2
    Tse_grasp = Tsc_initial.dot(Tce_grasp)
    # generate trajectory when approaching the grasping position in segment 2
    Tse_seg2 = mr.CartesianTrajectory(segment1[-1],Tse_grasp,2,2/0.01,3)
    # append the trajectory of segment 2 after segment 1
    Tse_before = np.append(segment1,Tse_seg2,axis=0)

    ##################################
    # segment 3: Closing of the gripper
    # append the trajectory of segment 3 by 63 times
    for i in np.arange(64):
        Tse_before = np.append(Tse_before,np.array([Tse_before[-1]]),axis=0)
    
    ##################################
    # segment 4: A trajectory to move the gripper back up to the "standoff" configuration
    # generate trajectory when back on the standoff position in segment 4
    Tse_seg4 = mr.CartesianTrajectory(Tse_grasp,Tse_standoff_before,2,2/0.01,3)
    # append the trajectory of segment 4
    Tse_before = np.append(Tse_before,Tse_seg4,axis=0)

    ##################################
    # segment 5: A trajectory to move the gripper to a "standoff" configuration above the final configuration
    # generate trajectory when moving to the final standoff position in segment 5
    Tse_standoff_after = Tsc_final.dot(Tce_standoff)
    Tse_seg5 = mr.CartesianTrajectory(Tse_standoff_before,Tse_standoff_after,8,8/0.01,3)
    # append the trajectory of segment 5
    Tse_before = np.append(Tse_before,Tse_seg5,axis=0)

    ##################################
    # segment 6: A trajectory to move the gripper to the final configuration of the object
    # generate the end-effector configuration when losing
    Tse_lose = Tsc_final.dot(Tce_grasp)
    # generate trajectory when moving to the final cube position in segment 6
    Tse_seg6 = mr.CartesianTrajectory(Tse_standoff_after,Tse_lose,2,2/0.01,3)
    # append the trajectory of segment 6
    Tse_before = np.append(Tse_before,Tse_seg6,axis=0)

    ##################################
    # segment 7: Opening of the gripper
    # append the trajectory of segment 7 by 63 times
    for i in np.arange(64):
        Tse_before = np.append(Tse_before,np.array([Tse_before[-1]]),axis=0)
    
    ##################################
    # segment 8: A trajectory to move the gripper back to the "standoff" configuration
    # generate trajectory when moving to the final standoff position in segment 8
    Tse_seg8 = mr.CartesianTrajectory(Tse_before[-1],Tse_standoff_after,2,2/0.01,3)
    # append the trajectory of segment 8
    Tse_before = np.append(Tse_before,Tse_seg8,axis=0)

    ##################################
    # generate a matrix which is n by 13
    Tse_post = np.zeros([int(k*21/0.01+64*2),13])
    # put the configuration, position and gripper state in matrix which is n by 13
    for i in np.arange(int(k*21/0.01+64*2)):
        Tse_post[i,0] = Tse_before[i,0,0]
        Tse_post[i,1] = Tse_before[i,0,1]
        Tse_post[i,2] = Tse_before[i,0,2]
        Tse_post[i,3] = Tse_before[i,1,0]
        Tse_post[i,4] = Tse_before[i,1,1]
        Tse_post[i,5] = Tse_before[i,1,2]
        Tse_post[i,6] = Tse_before[i,2,0]
        Tse_post[i,7] = Tse_before[i,2,1]
        Tse_post[i,8] = Tse_before[i,2,2]
        Tse_post[i,9] = Tse_before[i,0,3]
        Tse_post[i,10] = Tse_before[i,1,3]
        Tse_post[i,11] = Tse_before[i,2,3]
        Tse_post[i,12] = 0
    # amend the gripper state in segment 3, 4, 5, 6
    for i in np.arange(int(k*7/0.01),int(k*19/0.01+64)):
        Tse_post[i,12] = 1

    np.savetxt("traj_output.csv", Tse_post, delimiter=',')
    return Tse_post

if __name__ == '__main__':
    Tse_initial = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])
    Tsc_initial = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])
    Tsc_final = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1, 0.025],[0,0,0,1]])
    Tce_grasp = np.array([[-np.sqrt(2)/2,0,np.sqrt(2)/2,0],[0,1,0,0],[-np.sqrt(2)/2,0,-np.sqrt(2)/2,0],[0,0,0,1]])
    Tce_standoff = np.array([[-np.sqrt(2)/2,0,np.sqrt(2)/2,0],[0,1,0,0],[-np.sqrt(2)/2,0,-np.sqrt(2)/2,0.1],[0,0,0,1]])
    k = 1
    Tse_post = TrajectoryGenerator(Tse_initial,Tsc_initial,Tsc_final,Tce_grasp,Tce_standoff,k)
