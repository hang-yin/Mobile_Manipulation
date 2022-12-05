1. File Organization
- All the pythons files required to run this project are in the /code directory. 
- To check the result for the 3 conditions (best, overshoot, new task), checkout the three directories: /best, /overshoot, /new_task.
- In each of the three directories, you will find a README.txt describing that condition, a csv file for reproducing the behavior in CoppeliaSim, a video of this behavior in simulation, the X error data file, a plot of the error data, and a log file for running the corresponding code. 

2. Running Code
- To run the experiments, check out the final_script.py file in the /code directory. 
- You will need to comment/uncomment some of the code in this file to run the 3 different conditions described earlier. 
- There are four functions defined in the other python files: NextState, FeedbackControl, TrajectoryGenerator, testJointLimits. 
- To understand these functions, check out their respective code and comment

3. Joint Limit Implementation
- To account for singularities and self-collision, I implemented the function called testJointLimits. 
- Given the robot's current configuration, this function returns a list of joints that have violated the joint constraints we defined. 
- To help the robot arm avoid singularities, I constrained both joint 3 and joint 4 to be always less than -0.3. 
- I chose -0.3 because a number close to zero would make the difference between considering joint limits and without less visible. 
- Joint 1 and Joint 5 are less relevant with joint limits as they can hardly cause self collision. However, there should still be joint limits on them given their mechanical design.  
- In the /best directory, there are two videos: best.avi and best_without_joint_limits.avi. 
- From the videos, we can see that when we are not considering joint limits, the robot arm would approach singularities during the pick and place task. 
- However, when we do consider joint limits, the robot arm configuration would be further from singularities. 