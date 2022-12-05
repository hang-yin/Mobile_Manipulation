import numpy as np
import modern_robotics as mr

def testJointLimits(X):
    result = []
    # to avoid singularity, we need to make sure that the 3rd and 4th arm joints are not too close to 0
    # let's use 0.3 as the threshold for now
    if X[0,5] > -0.3:
        result.append(2)
    if X[0,6] > -0.3:
        result.append(3)
    return result