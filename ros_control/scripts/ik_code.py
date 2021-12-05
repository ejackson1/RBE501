from modern_robotics.core import JacobianSpace
import functions
import math
import numpy as np
import modern_robotics as mr

lambda = 0.1

def rot(x,y,z):
    t1 = np.radians(x)
    c1= np.cos(t1)
    s1 = np.sin(t1)
    R1 = np.array( ((c1, -s1, 0), (s1,c1,0), (0,0,1)))
    t2 = np.radians(y)
    c2= np.cos(t2)
    s2 = np.sin(t2)
    R2 = np.array(((c2, 0, s2), (0,1,0), (-s2,0,c2)))
    t3 = np.radians(z)
    c3= np.cos(t3)
    s3 = np.sin(t3)
    R3 = np.array(((1,0,0), (0,c3,s3), (0,s3,c3)))
    R = np.multiply(R1,R2,R3)
    return R

def transformation(x,y,z,a,b,c):
    R = rot(a,b,c)
    p = np.array(a,b,c)
    Tr = np.matrix(R, p)
    T = np.vstack(Tr;(0,0,0,1))
     
#acquiring values from topics

#from joint state publisher
currentQ=np.zeros(1,7) 

#from xbox controller
targetX, targetY, targetZ = 0,0,0 
roll,pitch,yaw=0,0,0 
T_target=transformation(targetX,targetY,targetZ,roll,pitch,yaw) # Wite a funnction that does this

#Calculate the poses
T_current=mr.FKinSpace(S,M,currentQ)
currentPose = mr.MatrixLog6(T_current)
currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']'
targetpose = mr.MatrixLog6(T_target)
nTests = 100 #some random variable
# error = []
# iteration = []
# qList = zeros(36,7)

while norm(targetpose(:,i) - currentPose) > ie-3:
    J = JacobianSpace(S,currentQ)
    deltaQ = (J'*np.inverse(J*J'+lambda^2*eye(6)))*(targetPose(:,i)-currentPose)
    currentQ = currentQ + deltaQ'
    error(end+1) = norm(targetpose(:,i))

