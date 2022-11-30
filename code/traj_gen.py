import numpy as np
import modern_robotics as mr

def TrajectoryGenerator(Tse_initial,Tsc_initial,Tsc_final,Tce_grasp,Tce_standoff,k):

    time_dict = {"1": 6.0,
                 "2": 3.0,
                 "3": 0.64,
                 "4": 3.0,
                 "5": 10.0,
                 "6": 3.0,
                 "7": 0.64,
                 "8": 3.0,}
    time_sum = time_dict["1"] + time_dict["2"] + time_dict["4"] + time_dict["5"] + time_dict["6"] + time_dict["8"]

    time_step = 0.01
    """
    Segment 1: Move from initial to the pre-grasp configuration
    Time: 5.0 seconds
    """
    Tse_standoff_before = Tsc_initial.dot(Tce_standoff)
    segment1 = mr.CartesianTrajectory(Tse_initial,Tse_standoff_before,time_dict["1"],time_dict["1"]/time_step,3)
    
    """
    Segment 2: Move from pre-grasp to grasp configuration
    Time: 2.0 seconds
    """
    Tse_grasp = Tsc_initial.dot(Tce_grasp)
    segment2 = mr.CartesianTrajectory(segment1[-1],Tse_grasp,time_dict["2"],time_dict["2"]/time_step,3)
    my_trajectory = np.append(segment1,segment2,axis=0)

    """
    Segment 3: Close the gripper
    Time: 0.64 seconds (should be 0.625 seconds but whatever)
    """
    for _ in np.arange(int(time_dict["3"]/time_step)):
        # keeps the same end effector position, we will add the gripper entry later
        my_trajectory = np.append(my_trajectory,np.array([my_trajectory[-1]]),axis=0)
    
    """
    Segment 4: Move from grasp back to pre-grasp configuration
    Time: 2.0 seconds
    """
    segment4 = mr.CartesianTrajectory(Tse_grasp,Tse_standoff_before,time_dict["4"],time_dict["4"]/time_step,3)
    my_trajectory = np.append(my_trajectory,segment4,axis=0)

    """
    Segment 5: Move from pre-grasp to "standoff" of final configuration
    Time: 8.0 seconds
    """
    Tse_standoff_after = Tsc_final.dot(Tce_standoff)
    segment5 = mr.CartesianTrajectory(Tse_standoff_before,Tse_standoff_after,time_dict["5"],time_dict["5"]/time_step,3)
    my_trajectory = np.append(my_trajectory,segment5,axis=0)

    """
    Segment 6: Move from "standoff" to final configuration
    Time: 2.0 seconds
    """
    segment6 = mr.CartesianTrajectory(Tse_standoff_after,Tsc_final.dot(Tce_grasp),time_dict["6"],time_dict["6"]/time_step,3)
    my_trajectory = np.append(my_trajectory,segment6,axis=0)

    """
    Segment 7: Open the gripper, similar to segment 3
    Time: 0.64 seconds
    """
    for _ in np.arange(int(time_dict["7"]/time_step)):
        my_trajectory = np.append(my_trajectory,np.array([my_trajectory[-1]]),axis=0)
    
    """
    Segment 8: Move from grasp back to pre-grasp configuration
    Time: 2.0 seconds
    """
    segment8 = mr.CartesianTrajectory(my_trajectory[-1],Tse_standoff_after,time_dict["8"],time_dict["8"]/time_step,3)
    my_trajectory = np.append(my_trajectory,segment8,axis=0)

    """
    Build the final trajectory
    """
    matrix_size = [int(k*time_sum/time_step+time_dict["3"]+time_dict["7"]),13]
    final_trajectory = np.zeros(matrix_size)
    for i in np.arange(int(k*time_sum/time_step+time_dict["3"]+time_dict["7"])):
        # flatten the trajectory to fit the output format
        final_trajectory[i,0] = my_trajectory[i,0,0]
        final_trajectory[i,1] = my_trajectory[i,0,1]
        final_trajectory[i,2] = my_trajectory[i,0,2]
        final_trajectory[i,3] = my_trajectory[i,1,0]
        final_trajectory[i,4] = my_trajectory[i,1,1]
        final_trajectory[i,5] = my_trajectory[i,1,2]
        final_trajectory[i,6] = my_trajectory[i,2,0]
        final_trajectory[i,7] = my_trajectory[i,2,1]
        final_trajectory[i,8] = my_trajectory[i,2,2]
        final_trajectory[i,9] = my_trajectory[i,0,3]
        final_trajectory[i,10] = my_trajectory[i,1,3]
        final_trajectory[i,11] = my_trajectory[i,2,3]
        final_trajectory[i,12] = 0
    # change the gripper state from segment 3 to 6
    gripper_closed_range = np.arange(int(k*(time_dict["1"]+time_dict["2"])/time_step),
                                     int(k*(time_sum-time_dict["8"])/time_step+time_dict["3"]))
    for i in gripper_closed_range:
        final_trajectory[i,12] = 1

    # np.savetxt("traj_output.csv", final_trajectory, delimiter=',')
    return final_trajectory
