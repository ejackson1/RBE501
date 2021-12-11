from modern_robotics.core import JacobianSpace
from numpy.lib.function_base import copy
from numpy.linalg import inv, norm
import functions
import math
import numpy as np
import modern_robotics as mr

L = 0.1

def rot(x,y,z):
    t1 = np.radians(x)
    c1= np.cos(t1)
    s1 = np.sin(t1)
    R1 = np.matrix( [[c1, -s1, 0], [s1,c1,0], [0,0,1]])
    t2 = np.radians(y)
    c2= np.cos(t2)
    s2 = np.sin(t2)
    R2 = np.matrix([[c2, 0, s2], [0,1,0], [-s2,0,c2]])
    t3 = np.radians(z)
    c3= np.cos(t3)
    s3 = np.sin(t3)
    R3 = np.matrix([[1,0,0], [0,c3,s3], [0,s3,c3]])
    R = np.multiply(R1,R2,R3)
    return R

def transformation(x,y,z,a,b,c):
    R = rot(a,b,c)
    p = np.matrix([a,b,c])
    zero=np.matrix([0,0,0,1])
    T1 = np.concatenate((R,p.T),axis=1)
    T=np.concatenate((T1,zero),axis=0)
    return T
    
def T2twist(T):
    CPose = mr.MatrixLog6(T.T)
    Pose = np.matrix([CPose(1,0), CPose(0,2), CPose(2,1), CPose(0,3), CPose(1,3), CPose(2,3)])
    return Pose

def deltaQ(J,x,targetPose,currentPose):
    #define the damped least squares formula
    dQ = J.T * inv(J * J.T + (L**2) * np.eye(3)) * (targetPose - currentPose)
    return dQ

#acquiring values from topics

#from joint state publisher
currentQ=np.zeros(1,7) 

#from xbox controller
targetX, targetY, targetZ = 0,0,0 
roll,pitch,yaw=0,0,0 
T_target=transformation(targetX,targetY,targetZ,roll,pitch,yaw) # Wite a funnction that does this

#Calculate the poses
T_current=mr.FKinSpace(S,M,currentQ)
currentPose = T2twist(T_current)
targetPose=T2twist(T_target)

cQ = currentQ.copy
#inverse kinematics calculations
while np.linalg.norm(targetPose - currentPose) > ie-3:
    J = mr.JacobianSpace(S,currentQ).T  
    deltaQ = deltaQ(J,x,targetPose,currentPose)
    cQ = cQ + deltaQ.T
    T_approaching = mr.FKinSpace(M,S,cQ)
    Approaching_Pose = T2twist(T_approaching) 

#send cQ to the joint position controller topic
