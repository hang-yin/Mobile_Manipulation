import numpy as np
import modern_robotics as mr
import copy

# define robot parameters

r = 0.0475 # wheel radius
l = 0.235 # half of distance between wheels
w = 0.15 # half of width of the robot

M0e = np.array([[1,0,0,0.033],
              [0,1,0,0],
              [0,0,1,0.6546],
              [0,0,0,1]])

Blist = np.array([[0,0,1,0,0.033,0],
                  [0,-1,0,-0.5076,0,0],
                  [0,-1,0,-0.3526,0,0],
                  [0,-1,0,-0.2176,0,0],
                  [0,0,1,0,0,0]]).T

Tb0 = np.array([[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]])



def FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt):
    
    # calculate X1
    theta_i = np.array([X[0,3],X[0,4],X[0,5],X[0,6],X[0,7]])
    # construct transformation matrix with the angle and position vector in X
    Tsb_i = np.array([[np.cos(X[0,0]),-np.sin(X[0,0]),0,X[0,1]],
                      [np.sin(X[0,0]),np.cos(X[0,0]),0,X[0,2]],
                      [0,0,1,0.0963],
                      [0,0,0,1]])
    X1 = Tsb_i.dot(Tb0).dot(mr.FKinBody(M0e, Blist, theta_i))

    # calculate Vd and Vb
    Vd = mr.se3ToVec((1/dt)*mr.MatrixLog6(np.linalg.inv(Xd).dot(Xd_next)))
    Vb = mr.Adjoint(np.linalg.inv(X1).dot(Xd)).dot(Vd)

    # calculate X_err term
    X_err = mr.se3ToVec(mr.MatrixLog6(np.linalg.inv(X1).dot(Xd)))
    
    # calculate twist using the control law equation!
    V = Vb + Kp * X_err + Ki * (X_err + dt * X_err)

    # to find Je(theta), we need to find both J_arm and J_base

    # define J_arm
    J_arm = mr.JacobianBody(Blist,theta_i)

    # define F6
    temp = np.array([[np.cos(0),np.sin(0)],[-np.sin(0),np.cos(0)]])
    r1 = (((1/r)*np.array([1,np.tan(-np.pi/4)])).dot(temp)).dot(np.array([[-w,1,0],[l,0,1]]))
    r2 = (((1/r)*np.array([1,np.tan(np.pi/4)])).dot(temp)).dot(np.array([[w,1,0],[l,0,1]]))
    r3 = (((1/r)*np.array([1,np.tan(-np.pi/4)])).dot(temp)).dot(np.array([[w,1,0],[-l,0,1]]))
    r4 = (((1/r)*np.array([1,np.tan(np.pi/4)])).dot(temp)).dot(np.array([[-w,1,0],[-l,0,1]]))
    # get F and then put it into F6
    F = np.linalg.pinv(np.vstack((r1,r2,r3,r4)))
    F6 = np.array([[0,0,0,0],
                   [0,0,0,0],
                   [F[0,0],F[0,1],F[0,2],F[0,3]],
                   [F[1,0],F[1,1],F[1,2],F[1,3]],
                   [F[2,0],F[2,1],F[2,2],F[2,3]],
                   [0,0,0,0]])

    # define J_base given F6
    J_base = mr.Adjoint(np.linalg.inv(X1).dot(Tsb_i)).dot(F6)

    # define Je with J_base and J_arm
    Je = np.hstack((J_base,J_arm))

    # here we write down speeds (u,thetadot)
    commanded_speed = jacobian_pseudo_inverse(Je,V)

    return V, commanded_speed

def jacobian_pseudo_inverse(J, V):
    return np.linalg.pinv(J,rcond=1e-2).dot(V)

    """
    TODO: add joint limits
    if use_joint_limit:
        testJointLimits(config,J_arm)
    """
