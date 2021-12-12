import math
import numpy as np
import modern_robotics as mr

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

def analytical_Jacobian(Slist,thetalist,M):
    Js = mr.JacobianSpace(Slist,thetalist)
    T = mr.FKinSpace(M,Slist,thetalist)
    sp = mr.VecToso3(T[0:3,3])
    J_a=Js[3:6,:]-np.dot(sp,Js[0:3,:])
    return J_a 


def deltaQ(J,targetPose,currentPose,lamda=0.1):
    #defMatrixExp6ine the damped least squares formula
    A= J.T
    B=J @ J.T
    C=(lamda**2)*np.eye(3)
    D=(targetPose - currentPose)
    #print("\nA={}\nB={}\nC={}\nD={}".format(np.shape(A),np.shape(B),np.shape(C),np.shape(D)))
    dQ=A@np.linalg.inv(B+C)@D
    return dQ


def T2twist(T):
    CPose = mr.MatrixLog6(T)
    Pose = np.array([CPose[1,0], CPose[0,2], CPose[2,1], CPose[0,3], CPose[1,3], CPose[2,3]])
    return Pose

def transformation(x,y,z,a,b,c):
    R = rot(a,b,c)
    p = np.matrix([x,y,z])
    #print("Linear Cordinates (p): ", p)
    #print("\n")
    zero=np.matrix([0,0,0,1])
    T1 = np.concatenate((R,p.T),axis=1)
    T = np.array(np.concatenate((T1,zero),axis=0))
    return T

def iksolver_mr(x,y,z,currentQ,roll=0,pitch=0,yaw=0):
    T_target=transformation(x,y,z,roll,pitch,yaw) 
    finalQ = mr.IKinSpace(Slist, M, T_target, currentQ, eomg=0.01, ev=0.001)
    #print("MRJacobian = \n Tranformation matrix = {} \n joint_values = {}".format(mr.FKinSpace(M, Slist, finalQ),finalQ))
    return finalQ
    
#Initilize all values

Slist = np.array([[0, 0, -1, 0, 0, 0],
    [0,1, 0, -.2848, 0, 0],
    [0, 0, -1, 0.0118, 0, 0],
    [0, 1, 0, -.7056, 0, 0],
    [0, 0, -1, 0, 0.0245, 0],
    [0, 1, 0, -1.0199, 0, 0],
    [0, 0, -1, 0.0247, 0, 0]]).T

M = np.array([[1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, -.0249, 1.1854, 1]]).T;

currentQ= np.array([0, 0, 0, 0, 0, 0, 0]); 

def inverse_kinematics(x,y,z,currentQ,roll=0,pitch=0,yaw=0,Jacobian="analytical"):
    T_target=transformation(x,y,z,roll,pitch,yaw) 
    T_current=mr.FKinSpace(M,Slist,currentQ)
    cost,itr =0,0
    
    if Jacobian=="analytical":
        current_Pose  = T_current[0:3,3]
        target_Pose   = T_target[0:3,3]
        #inverse kinematics calculations
        while np.linalg.norm(target_Pose - current_Pose) > 0.001:   
            J_a = analytical_Jacobian(Slist,currentQ,M)
            dQ = deltaQ(J_a,target_Pose,current_Pose,lamda=0.001)
            currentQ = currentQ + dQ
            T_approaching = mr.FKinSpace(M,Slist,currentQ)
            current_Pose = T_approaching[0:3,3]
            cost = np.linalg.norm(target_Pose - current_Pose)
            itr = itr+1
            #print(cost,"\r")
            #print("Iterations = \r",itr)
    else:
        current_Pose = T2twist(T_current)
        target_Pose = T2twist(T_target)
        while np.linalg.norm(target_Pose - current_Pose) > 0.001:   
            Js = mr.JacobianSpace(Slist,currentQ)
            dQ = deltaQ(Js,target_Pose,current_Pose,lamda=0.1)
            currentQ = currentQ + dQ
            T_approaching = mr.FKinSpace(M,Slist,currentQ)
            current_Pose = T2twist(T_approaching)
            np.linalg.norm(target_Pose - current_Pose)
            cost = np.linalg.norm(target_Pose - current_Pose)
            itr = itr+1
    
    print("NativeJacobian = {} \n Tranformation matrix = {} \n joint_values = {}".format(Jacobian,mr.FKinSpace(M, Slist, currentQ),currentQ))
    #print(currentQ)        
    return currentQ

inverse_kinematics(0.1342, -0.0289, 1.1627, currentQ)
