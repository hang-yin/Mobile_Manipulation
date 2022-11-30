import numpy as np
import modern_robotics as mr
import copy

def NextState(curr_config,speed,dt,max_speed):
    """
    Input: 
    curr_config: a 12-vector representing current configuration of the robot
    speed: a 9-vector of joint speeds (including the wheels and the arm joints)
    dt: time step
    max_speed: maximum angular speed of the robot

    Output:
    next_config: a 12-vector representing the next configuration of the robot
    """

    # define constants: robot parameters
    r = 0.0475 # wheel radius
    l = 0.235 # half of distance between wheels
    w = 0.15 # half of width of the robot

    # limit the speed of the robot according to the max speed
    _,m = speed.shape
    for i in range(m):
        if speed[0,i] > max_speed:
            speed[0,i] = max_speed
        elif speed[0,i] < -max_speed:
            speed[0,i] = -max_speed

    # calculate the next configuration (not yet with odometry)
    arm_config = curr_config[0, 3:12] + speed*dt

    # calculate the next configuration with odometry
    # find change in wheel rotation
    dtheta = (speed[0,5:].reshape(1,4)).T * dt
    # find chassis twist
    temp = 1/(l+w)
    twist = (1/4)*r*np.array([[-temp,temp,temp,-temp],
                              [1,1,1,1],
                              [-1,1,-1,1]]).dot(dtheta)
    
    wbz = twist[0,0]
    vbx = twist[1,0]
    vby = twist[2,0]
    if wbz == 0:
        dqb = np.array([[0],
                        [vbx],
                        [vby]])
    else:
        dqb = np.array([[wbz], 
                        [(vbx * np.sin(wbz) + (vby * (np.cos(wbz)-1))) / wbz], 
                        [(vby * np.sin(wbz) + (vbx * (1 - np.cos(wbz)))) / wbz]])
    
    dq = np.array([[1,0,0], 
                   [0, np.cos(curr_config[0,0]), -np.sin(curr_config[0,0])], 
                   [0, np.sin(curr_config[0,0]), np.cos(curr_config[0,0])]]).dot(dqb)

    chassis_config = curr_config[0,0:3].reshape(1,3)+dq.reshape(1,3)

    result = np.hstack((chassis_config,arm_config))
    return result
