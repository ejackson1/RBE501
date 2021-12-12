#!/usr/bin/env python3

from numpy.core.fromnumeric import transpose
import rospy
import numpy as np

def skew(vector):
    #print(vector[0])
    Skew = np.array([[0, -vector[2], vector[1]],
                    [vector[2], 0, -vector[0]],
                    [-vector[1], vector[0], 0]])
    return Skew

def axisangle2rot(omega, theta):
    a = np.add(np.identity(3), np.multiply(np.sin(theta), skew(omega)))
    omega2 = np.matmul(skew(omega), skew(omega))
    b = np.multiply((np.subtract(1 ,(np.cos(theta)))), omega2)
    R = np.add(a,b)

    return R

def vcomp(omega, theta, v):
    a = np.multiply(np.identity(3), theta)
    b = np.multiply((1-np.cos(theta)), skew(omega))
    c = np.multiply((theta - np.sin(theta)), np.matmul(skew(omega),skew(omega)))
    d = np.add(np.add(a,b), c)

    vcomp = np.matmul(d, v)
    return vcomp

def twist2ht(S, theta):
        omega = S[0:3]
        v = S[3:6]
        R = axisangle2rot(omega, theta)
        vc = vcomp(omega, theta, v)
        vc = vc.reshape(3,1)
        T1 = np.hstack((R, vc))
        T2 = np.array([0, 0, 0, 1])
        T = np.vstack((T1, T2))

        return T

def fkine(S, M, q):

    sumT = np.identity(4)
    index = 0
    for joint_ang in q:
        SingleS = S[:,index]
        #SingleS = SingleSCol.reshape(1,6)
        # omega = SingleS[0:3]
        # v = SingleS[3:6]
        # R = axisangle2rot(omega, joint_ang)
        # vc = vcomp(omega, joint_ang, v)
        # vc = vc.reshape(3,1)
        # T1 = np.hstack((R, vc))
        # T2 = np.array([0, 0, 0, 1])
        # T = np.vstack((T1, T2))
        T = twist2ht(SingleS, joint_ang)

        sumT = np.matmul(sumT, T)
        index += 1

    Tfinal = np.matmul(sumT, M)

    return Tfinal

def jacoba(S, M, currentQ):

    def adjoint(T):
        R = T[0:3,0:3]
        P = T[0:3,3]
        zeros = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
        adjTop = np.hstack((R, zeros))
        adjBot = np.hstack(((np.matmul(skew(P), R)), R))
        return np.vstack((adjTop, adjBot))
    
    sumT = np.identity(4)
    index = 0
    Jacobian = np.zeros((6, 7))
    for joint_ang in currentQ:
        SingleS = S[:,index]
        #SingleS = SingleSCol.reshape(1,6)
        T = twist2ht(SingleS, joint_ang)
        sumT = np.matmul(sumT, T)
        sAdjoint = adjoint(sumT)
        sJ = np.matmul(sAdjoint, SingleS)
        x = 0
        for value in sJ:
            Jacobian[x,index] = value
            x += 1

        index += 1
    #print(Jacobian)
    fk = fkine(S, M, q)
    bracketP = skew(fk[0:3,3])
    Jsw = Jacobian[0:3,:]
    Jsv = Jacobian[3:6,:]

    analJacobian = np.subtract(Jsv, np.matmul(bracketP, Jsw))
    #print(analJacobian)
    return analJacobian

def ik(current_Pose, target_Pose, currentQ, S, M):
    fixCurrentQ = np.array(np.zeros((1,7)))

    print(current_Pose)
    print(target_Pose)
    print(currentQ)

    #rospy.sleep(10)
    while np.linalg.norm(target_Pose - current_Pose) > 0.001:
        J_a = jacoba(S, M, currentQ)
        # lamda = 0.5
        pose_diff = np.subtract(target_Pose, current_Pose)
        #print(pose_diff)
        # mid = np.linalg.inv((np.matmul(J_a, np.transpose(J_a))) + np.multiply(np.multiply(lamda, lamda), np.identity(3)))
        # deltaQ = np.matmul(np.matmul(np.transpose(J_a), mid), pose_diff)
        #print(deltaQ)

        deltaQ = np.matmul(np.linalg.pinv(J_a), pose_diff)
        print(deltaQ)
        currentQ = deltaQ + currentQ.reshape(7,1)
        #currentQ = currentQ.reshape(1,7)
        #print(currentQ)
        # index = 0
        # for value in currentQ:
        #     #print(value)
        #     fixCurrentQ[0:index] = value
            
        #     index += 1
        # print(fixCurrentQ)
        print("a")
        T = fkine(S, M, currentQ)
        print("T " + str(T))
        current_Pose = T[0:3,3]
        current_Pose = current_Pose.reshape(3,1)
        print("currentPose " + str(current_Pose))

    return currentQ


L0 = 154.6 / 1000
L1 = 128.4 / 1000
L2 = 210.4 / 1000
L3 = L2
L4 = L2
L5 = 105.9 / 1000
L6 = L5
L7 = 61.5 / 1000

a1 = 5.4 / 1000
a2 = 6.4 / 1000
a3 = a2
a4 = a2

w1 = np.array([0, 0, -1])
p1 = np.array([0, 0, L0])
v1 = np.matmul(skew(w1), -p1)
S1 = np.vstack((w1.reshape(3,1), v1.reshape(3,1)))

w2 = np.array([0, 1, 0])
p2 = np.array([0, -a1, L0 + L1])
v2 = np.matmul(skew(w2), -p2)
S2 = np.vstack((w2.reshape(3,1), v2.reshape(3,1)))


w3 = np.array([0, 0, -1])
p3 = np.array([0, -(a1 + a2), L0 + L1 + L2])
v3 = np.matmul(skew(w3), -p3)
S3 = np.vstack((w3.reshape(3,1), v3.reshape(3,1)))

w4 = np.array([0, 1, 0])
p4 = np.array([0, -(a1 + a2 + a3), L0 + L1 + L2 + L3])
v4 = np.matmul(skew(w4), -p4)
S4 = np.vstack((w4.reshape(3,1), v4.reshape(3,1)))

w5 = np.array([0, 0, -1])
p5 = np.array([0, -(a1 + a2 + a3 + a4), L0 + L1 + L2 + L3 + L4])
v5 = np.matmul(skew(w5), -p5)
S5 = np.vstack((w5.reshape(3,1), v5.reshape(3,1)))

w6 = np.array([0, 1, 0])
p6 = np.array([0, -(a1 + a2 + a3 + a4), L0 + L1 + L2 + L3 + L4 + L5])
v6 = np.matmul(skew(w6), -p6)
S6 = np.vstack((w6.reshape(3,1), v6.reshape(3,1)))

w7 = np.array([0, 0, -1])
p7 = np.array([0, -(a1 + a2 + a3 + a4), L0 + L1 + L2 + L3 + L4 + L5 + L6])
v7 = np.matmul(skew(w7), -p7)
S7 = np.vstack((w7.reshape(3,1), v7.reshape(3,1)))

S = np.hstack((S1, S2, S3, S4, S5, S6, S7))
#print(S)
Mr = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
Mp = np.array([[0], [-(a1 + a2 + a3 + a4)], [L0 + L1 + L2 + L3 + L4 + L5 + L6 + L7]])
Mtop = np.hstack((Mr, Mp))
M = np.vstack((Mtop, [0, 0, 0, 1]))

q = np.array([0, 0, 0, 0, 0, 1.1, 0])
#print(q)
#print(S[:,0])
T = fkine(S, M, q.reshape(7,1))
#print(T)

#Jacobian = jacoba(S,M,q)

current_Pose = fkine(S, M, q)[0:3,3]
current_Pose = current_Pose.reshape(3,1)
#print(fkine(S, M, q))
target_Pose = np.array([[0.5], [0.5], [0.5]])
print(ik(current_Pose, target_Pose, q, S, M))


#print(quaternion_rotation_matrix(np.array([0, 0, 0, -1])))

#print(vcomp(np.array([1,2,3]), np.array([1]), np.array([[1], [1], [1]])))

#print(fkine(np.array([1, 0, 0, 0, 0, 1]), np.array([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]]), np.array([[2]])))