#!/usr/bin/env python3

import sys, select, termios, tty
import rospy
import numpy as np
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState
from ros_control.srv import jointTraj
import matplotlib.pyplot as plt
import modern_robotics as mr


class inverseKinematics:
    def __init__(self) -> None: 
        rospy.loginfo("Initalizing inverse kinematics node")
        rospy.Subscriber("/my_gen3/joint_states", JointState, self.joint_callback)

        rospy.init_node("ik_Node", anonymous=True)

        pass
    

    def joint_callback(self, joint_state):
        self.currentQ = joint_state.position
        self.currentV = joint_state.velocity

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def skew(self, vector):
        #print(vector[0])
        Skew = np.array([[0, -vector[2], vector[1]],
                        [vector[2], 0, -vector[0]],
                        [-vector[1], vector[0], 0]])
        return Skew

    def axisangle2rot(self, omega, theta):
        a = np.add(np.identity(3), np.multiply(np.sin(theta), self.skew(omega)))
        omega2 = np.matmul(self.skew(omega), self.skew(omega))
        b = np.multiply((np.subtract(1 ,(np.cos(theta)))), omega2)
        R = np.add(a,b)

        return R

    def vcomp(self, omega, theta, v):
        a = np.multiply(np.identity(3), theta)
        b = np.multiply((1-np.cos(theta)), self.skew(omega))
        c = np.multiply((theta - np.sin(theta)), np.matmul(self.skew(omega),self.skew(omega)))
        d = np.add(np.add(a,b), c)

        vcomp = np.matmul(d, v)
        return vcomp

    def twist2ht(self, S, theta):
            omega = S[0:3]
            v = S[3:6]
            R = self.axisangle2rot(omega, theta)
            vc = self.vcomp(omega, theta, v)
            vc = vc.reshape(3,1)
            T1 = np.hstack((R, vc))
            T2 = np.array([0, 0, 0, 1])
            T = np.vstack((T1, T2))

            return T

    def fkine(self, S, M, q):

        sumT = np.identity(4)
        index = 0
        for joint_ang in q:
            SingleS = S[:,index]
            T = self.twist2ht(SingleS, joint_ang)

            sumT = np.matmul(sumT, T)
            index += 1

        Tfinal = np.matmul(sumT, M)

        return Tfinal

    def jacoba(self, S, M, currentQ):

        def adjoint(T):
            R = T[0:3,0:3]
            P = T[0:3,3]
            zeros = np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
            adjTop = np.hstack((R, zeros))
            adjBot = np.hstack(((np.matmul(self.skew(P), R)), R))
            return np.vstack((adjTop, adjBot))
        
        sumT = np.identity(4)
        index = 0
        Jacobian = np.zeros((6, 7))
        for joint_ang in currentQ:
            SingleS = S[:,index]
            #SingleS = SingleSCol.reshape(1,6)
            T = self.twist2ht(SingleS, joint_ang)
            sumT = np.matmul(sumT, T)
            sAdjoint = adjoint(sumT)
            sJ = np.matmul(sAdjoint, SingleS)
            x = 0
            for value in sJ:
                Jacobian[x,index] = value
                x += 1

            index += 1
        #print(Jacobian)
        fk = self.fkine(S, M, q)
        bracketP = self.skew(fk[0:3,3])
        Jsw = Jacobian[0:3,:]
        Jsv = Jacobian[3:6,:]

        analJacobian = np.subtract(Jsv, np.matmul(bracketP, Jsw))
        #print(analJacobian)
        return analJacobian

    
    def ik(self, current_Pose, target_Pose, currentQ, S, M):
        #rospy.sleep(10)
        index = 0
        while np.linalg.norm(target_Pose - current_Pose) > 0.001:
            J_a = self.jacoba(S, M, currentQ)
            #print(J_a)
            print(np.linalg.norm(target_Pose - current_Pose))
            #lamda = 0.5
            pose_diff = np.subtract(target_Pose, current_Pose)
            #print(pose_diff)
            #mid = np.linalg.inv((np.matmul(J_a, np.transpose(J_a))) + np.multiply(np.multiply(lamda, lamda), np.identity(3)))
            #deltaQ = np.matmul(np.matmul(np.transpose(J_a), mid), pose_diff)
            #print(deltaQ)
            #print(currentQ)
            #print(pose_diff)
            deltaQ = np.matmul(np.linalg.pinv(J_a), pose_diff)

            #deltaQ = np.matmul(np.transpose(J_a), pose_diff)


            currentQ = deltaQ + currentQ.reshape(7,1)
            #currentQ = currentQ.reshape(1,7)
            #print(currentQ)
            #print("a")
            T = self.fkine(S, M, currentQ)
            #print("T " + str(T))
            current_Pose = T[0:3,3]
            current_Pose = current_Pose.reshape(3,1)
            #print(current_Pose)
            index +=1
            #print(index)
            #print("currentPose " + str(current_Pose))
        #print(currentQ)
        return currentQ

    def quinticpoly(self, t0, tf, q0, qf, qd0, qdf, qdd0, qddf):
        
        a = np.linalg.inv(np.array([[1, t0, t0**2, t0**3, t0**4, t0**5],
                                    [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
                                    [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
                                    [1, tf, tf**2, tf**3, tf**4, tf**5],
                                    [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                                    [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]]))
        b = np.array([[q0], [qd0], [qdd0], [qf], [qdf], [qddf]])

        c = np.matmul(a, b)
        return c
    
    # def transformation(self, x,y,z,a,b,c):
    #     R = rot(a,b,c)
    #     p = np.matrix([x,y,z])
    #     #print("Linear Cordinates (p): ", p)
    #     #print("\n")
    #     zero=np.matrix([0,0,0,1])
    #     T1 = np.concatenate((R,p.T),axis=1)
    #     T = np.array(np.concatenate((T1,zero),axis=0))
    #     return T
    
    # def inverse_kinematics(self, x,y,z,currentQ,roll=0,pitch=0,yaw=0,S, MJacobian="analytical"):
    #     T_target=self.transformation(x,y,z,roll,pitch,yaw) 
    #     T_current=mr.FKinSpace(M,S,currentQ)
    #     cost,itr =0,0

    #     current_Pose  = T_current[0:3,3]
    #     target_Pose   = T_target[0:3,3]
    #     #inverse kinematics calculations
    #     while np.linalg.norm(target_Pose - current_Pose) > 0.001:   
    #         J_a = analytical_Jacobian(Slist,currentQ,M)
    #         dQ = deltaQ(J_a,target_Pose,current_Pose,lamda=0.001)
    #         currentQ = currentQ + dQ
    #         T_approaching = mr.FKinSpace(M,Slist,currentQ)
    #         current_Pose = T_approaching[0:3,3]
    #         cost = np.linalg.norm(target_Pose - current_Pose)
    #         itr = itr+1

    #     return currentQ


    def jointProfiles(self, coefficients, time_spacing, t0, tf):
        a0 = coefficients[0]
        a1 = coefficients[1]
        a2 = coefficients[2]
        a3 = coefficients[3]
        a4 = coefficients[4]
        a5 = coefficients[5]
        qListnp = np.array([])
        qdListnp = np.array([])
        qddListnp = np.array([])
        times = np.linspace(t0, tf, num= time_spacing)   

        for t in times:
            q = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
            qd = a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
            qdd = 2*a2 + 6*a3*t + 12*a4*t**2 + 20*a5*t**4
            qListnp = np.hstack((qListnp, q))
            qdListnp = np.hstack((qdListnp, qd))
            qddListnp = np.hstack((qddListnp, qdd))

        # Turn on figures to show plots :)

        # plt.figure(1)
        # plt.subplot(311)
        # plt.plot(times, qListnp)
        # plt.title("Q")
        # plt.subplot(312)
        # plt.plot(times, qdListnp)
        # plt.title("Qd")
        # plt.subplot(313)
        # plt.plot(times, qddListnp)
        # plt.title("Qdd")
        # plt.show()
        return (qListnp, qdListnp, qddListnp, times)

   
    def jointTrajectory(self, goal_Pose, currentAngles, S, M, time_spacing, tf):
        
        T = self.fkine(S, M, currentAngles)
        currPose = T[0:3, 3]
        currPose = currPose.reshape(3,1)

        index = 0
        desiredJointAngs = self.ik(currPose, goal_Pose, currentAngles, S, M)
        #print(desiredJointAngs)

        qList = np.array(np.zeros((1,time_spacing)))
        qdList = np.array(np.zeros((1,time_spacing)))
        qddList = np.array(np.zeros((1,time_spacing)))
        timeList = []
        for angle in desiredJointAngs:
            
            quinticCofs = self.quinticpoly(0, 1, currentAngles[index], angle, 0, 0, 0, 0)
            (qListnp, qdListnp, qddListnp, times) = self.jointProfiles(quinticCofs, time_spacing, 0, tf)
            qList = np.vstack((qList, qListnp))
            qdList = np.vstack((qdList, qdListnp))
            qddList = np.vstack((qddList, qddListnp))
            
            index += 1
        
        # Remove initial zeros 
        qList = np.delete(qList, 0, 0)
        qdList = np.delete(qdList, 0, 0)
        qddList = np.delete(qddList, 0, 0)

        # Turn numpy array into floating points of lists for each joint
        q1 = [float(element) for element in qList[0,:]]
        q2 = [float(element) for element in qList[1,:]]
        q3 = [float(element) for element in qList[2,:]]
        q4 = [float(element) for element in qList[3,:]]
        q5 = [float(element) for element in qList[4,:]]
        q6 = [float(element) for element in qList[5,:]]
        q7 = [float(element) for element in qList[6,:]]

        qd1 = [float(element) for element in qdList[0,:]]
        qd2 = [float(element) for element in qdList[1,:]]
        qd3 = [float(element) for element in qdList[2,:]]
        qd4 = [float(element) for element in qdList[3,:]]
        qd5 = [float(element) for element in qdList[4,:]]
        qd6 = [float(element) for element in qdList[5,:]]
        qd7 = [float(element) for element in qdList[6,:]]

        qdd1 = [float(element) for element in qddList[0,:]]
        qdd2 = [float(element) for element in qddList[1,:]]
        qdd3 = [float(element) for element in qddList[2,:]]
        qdd4 = [float(element) for element in qddList[3,:]]
        qdd5 = [float(element) for element in qddList[4,:]]
        qdd6 = [float(element) for element in qddList[5,:]]
        qdd7 = [float(element) for element in qddList[6,:]]

        # Shift times list over by one increment
        timeList = [float(element + times[1]) for element in times]
        print("here")
        #rospy.wait_for_service('/quintic', jointTraj)

        try:
            c = rospy.ServiceProxy('/quintic', jointTraj)
            success = c(q1, q2, q3, q4, q5, q6, q7,
                        qd1, qd2, qd3, qd4, qd5, qd6, qd7, 
                        qdd1, qdd2, qdd3, qdd4, qdd5, qdd6, qdd7, timeList)
            bool = Bool()
            bool.data = True            
            if success.data == bool:
                return
            else:
                return print(str(success.data.data) + ", unable to find a solution.")
        except Exception as e:
            print(e)


        return



if __name__ == "__main__":

    ik = inverseKinematics()

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
    v1 = np.matmul(ik.skew(w1), -p1)
    S1 = np.vstack((w1.reshape(3,1), v1.reshape(3,1)))

    w2 = np.array([0, 1, 0])
    p2 = np.array([0, -a1, L0 + L1])
    v2 = np.matmul(ik.skew(w2), -p2)
    S2 = np.vstack((w2.reshape(3,1), v2.reshape(3,1)))


    w3 = np.array([0, 0, -1])
    p3 = np.array([0, -(a1 + a2), L0 + L1 + L2])
    v3 = np.matmul(ik.skew(w3), -p3)
    S3 = np.vstack((w3.reshape(3,1), v3.reshape(3,1)))

    w4 = np.array([0, 1, 0])
    p4 = np.array([0, -(a1 + a2 + a3), L0 + L1 + L2 + L3])
    v4 = np.matmul(ik.skew(w4), -p4)
    S4 = np.vstack((w4.reshape(3,1), v4.reshape(3,1)))

    w5 = np.array([0, 0, -1])
    p5 = np.array([0, -(a1 + a2 + a3 + a4), L0 + L1 + L2 + L3 + L4])
    v5 = np.matmul(ik.skew(w5), -p5)
    S5 = np.vstack((w5.reshape(3,1), v5.reshape(3,1)))

    w6 = np.array([0, 1, 0])
    p6 = np.array([0, -(a1 + a2 + a3 + a4), L0 + L1 + L2 + L3 + L4 + L5])
    v6 = np.matmul(ik.skew(w6), -p6)
    S6 = np.vstack((w6.reshape(3,1), v6.reshape(3,1)))

    w7 = np.array([0, 0, -1])
    p7 = np.array([0, -(a1 + a2 + a3 + a4), L0 + L1 + L2 + L3 + L4 + L5 + L6])
    v7 = np.matmul(ik.skew(w7), -p7)
    S7 = np.vstack((w7.reshape(3,1), v7.reshape(3,1)))

    S = np.hstack((S1, S2, S3, S4, S5, S6, S7))
    #print(S)
    Mr = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    Mp = np.array([[0], [-(a1 + a2 + a3 + a4)], [L0 + L1 + L2 + L3 + L4 + L5 + L6 + L7]])
    Mtop = np.hstack((Mr, Mp))
    M = np.vstack((Mtop, [0, 0, 0, 1]))

    q = np.array([0, 0.26, 3.14, -2.27, 0, 0.95, 1.57])
    #q = np.array([0, -1.1, -1.1, 0, 0, 1.1, 0])

    #print(M)
    #print(S)
    q = np.array(ik.currentQ)
    print(q)

    #print(S[:,0])
    T = ik.fkine(S, M, q.reshape(7,1))
    print(T)

    #Jacobian = jacoba(S,M,q)
    #print(ik.jacoba(S,M,q.reshape(7,1)))

    current_Pose = ik.fkine(S, M, q)[0:3,3]
    current_Pose = current_Pose.reshape(3,1)
    #print(fkine(S, M, q))
    target_Pose = np.array([[0.3], [0.4], [0.5]])
    #print(ik.ik(current_Pose, target_Pose, q, S, M))

    #print(quinticpoly(0, 1, 0.4, 0.7, 0, 0, 0, 0))

    #ik.jointProfiles(ik.quinticpoly(0, 1, 0.4, 0.7, 0, 0, 0, 0), 100, 0, 1)
    tf = 1
    time_spacing = 10
    #ik.jointTrajectory(target_Pose, q, S, M, 30, tf)
    #jointTrajectory(self, goal_Pose, currentAngles, S, M, time_spacing):

    msg = """
    Control Your Arm!
    ---------------------------
    Moving around:
    i j k : control direction
    r p y : control angular orientation
    t/b : control positive (t) or negative (b)

    q : adjust positional increments by 10cm
    e : adjust positional increments by 1 cm
    w : adjust angular position by 1deg
    x : adjust angular position by 15deg

    m to quit
    """

    settings = termios.tcgetattr(sys.stdin)    

    directionBindings = ['i', 'j', 'k']
    angularBindings = ['r', 'p', 'y']
    posNegBindings = ['t', 'b']
    positionalBindings = ['q', 'e']
    angPositionalBindings = ['w', 'x']
    
    posNeg = 1
    direction = 'i'
    deg = 1
    T = ik.fkine(S, M, q.reshape(7,1))
    pose_goal = T[0:3,3].reshape(3,1)

    ## Teleop ##
    try:
        print(msg) 
        while True:
            key = ik.getKey()
            if key in directionBindings:
                if key == 'i':
                    direction = 'i'
                elif key == 'j':
                    direction = 'j'
                elif key == 'k':
                    direction = 'k'

            elif key in positionalBindings:
                if key == 'q':
                    if direction == 'i':
                        pose_goal[0] += (0.1 * posNeg)
                        ik.jointTrajectory(pose_goal, np.array(ik.currentQ), S, M, time_spacing, tf)
                    elif direction == 'j':
                        pose_goal[1] += (0.1 * posNeg)
                        ik.jointTrajectory(pose_goal, np.array(ik.currentQ), S, M, time_spacing, tf)
                    elif direction == 'k':
                        pose_goal[2] += (0.1 * posNeg)
                        ik.jointTrajectory(pose_goal, np.array(ik.currentQ), S, M, time_spacing, tf)
                
                elif key == 'e':
                    if direction == 'i':
                        pose_goal[0] += (0.01 * posNeg)
                        ik.jointTrajectory(pose_goal, np.array(ik.currentQ), S, M, time_spacing, tf)
                    elif direction == 'j':
                        pose_goal[1] += (0.01 * posNeg)
                        ik.jointTrajectory(pose_goal, np.array(ik.currentQ), S, M, time_spacing, tf) 
                    elif direction == 'k':
                        pose_goal[2] += (0.01 * posNeg)
                        ik.jointTrajectory(pose_goal, np.array(ik.currentQ), S, M, time_spacing, tf)
            
            
            # elif key in angularBindings:
            #     if key == 'r':
            #         if deg == 1:
            #             qx = pose_goal.orientation.x
            #             qy = pose_goal.orientation.y
            #             qz = pose_goal.orientation.z
            #             qw = pose_goal.orientation.w
            #             (roll, pitch, yaw) = euler_from_quaternion([qx, qy, qz, qw])
            #             roll += (posNeg * degs2rads(deg))                        
            #             q = quaternion_from_euler(roll, pitch, yaw, axes='sxyz')
            #             pose_goal.orientation.x = q[0]
            #             pose_goal.orientation.y = q[1]
            #             pose_goal.orientation.z = q[2]
            #             pose_goal.orientation.w = q[3]
            #             move_arm_client(pose_goal)
                    
            #         elif deg == 15:
            #             qx = pose_goal.orientation.x
            #             qy = pose_goal.orientation.y
            #             qz = pose_goal.orientation.x
            #             qw = pose_goal.orientation.w
            #             (roll, pitch, yaw) = euler_from_quaternion([qx, qy, qz, qw])
            #             roll += (posNeg * degs2rads(deg))                        
            #             q = quaternion_from_euler(roll, pitch, yaw, axes='sxyz')
            #             pose_goal.orientation.x = q[0]
            #             pose_goal.orientation.y = q[1]
            #             pose_goal.orientation.z = q[2]
            #             pose_goal.orientation.w = q[3]
            #             move_arm_client(pose_goal)

            #     elif key == 'p':
            #         if deg == 1:
            #             qx = pose_goal.orientation.x
            #             qy = pose_goal.orientation.y
            #             qz = pose_goal.orientation.z
            #             qw = pose_goal.orientation.w
            #             (roll, pitch, yaw) = euler_from_quaternion([qx, qy, qz, qw])
            #             pitch += (posNeg * degs2rads(deg))                        
            #             q = quaternion_from_euler(roll, pitch, yaw, axes='sxyz')
            #             pose_goal.orientation.x = q[0]
            #             pose_goal.orientation.y = q[1]
            #             pose_goal.orientation.z = q[2]
            #             pose_goal.orientation.w = q[3]
            #             move_arm_client(pose_goal)
            #         elif deg == 15:
            #             qx = pose_goal.orientation.x
            #             qy = pose_goal.orientation.y
            #             qz = pose_goal.orientation.z
            #             qw = pose_goal.orientation.w
            #             (roll, pitch, yaw) = euler_from_quaternion([qx, qy, qz, qw])
            #             pitch += (posNeg * degs2rads(deg))                        
            #             q = quaternion_from_euler(roll, pitch, yaw, axes='sxyz')
            #             pose_goal.orientation.x = q[0]
            #             pose_goal.orientation.y = q[1]
            #             pose_goal.orientation.z = q[2]
            #             pose_goal.orientation.w = q[3]
            #             move_arm_client(pose_goal)

            #     elif key == 'y':
            #         if deg == 1:
            #             qx = pose_goal.orientation.x
            #             qy = pose_goal.orientation.y
            #             qz = pose_goal.orientation.z
            #             qw = pose_goal.orientation.w
            #             (roll, pitch, yaw) = euler_from_quaternion([qx, qy, qz, qw])
            #             yaw += (posNeg * degs2rads(deg))                        
            #             q = quaternion_from_euler(roll, pitch, yaw, axes='sxyz')
            #             pose_goal.orientation.x = q[0]
            #             pose_goal.orientation.y = q[1]
            #             pose_goal.orientation.z = q[2]
            #             pose_goal.orientation.w = q[3]
            #             move_arm_client(pose_goal)
            #         elif deg == 15:
            #             qx = pose_goal.orientation.x
            #             qy = pose_goal.orientation.y
            #             qz = pose_goal.orientation.z
            #             qw = pose_goal.orientation.w
            #             (roll, pitch, yaw) = euler_from_quaternion([qx, qy, qz, qw])
            #             yaw += (posNeg * degs2rads(deg))                        
            #             q = quaternion_from_euler(roll, pitch, yaw, axes='sxyz')
            #             pose_goal.orientation.x = q[0]
            #             pose_goal.orientation.y = q[1]
            #             pose_goal.orientation.z = q[2]
            #             pose_goal.orientation.w = q[3]
            #             move_arm_client(pose_goal)


            elif key in posNegBindings:
                if key == 't':
                    posNeg = 1
                elif key == 'b':
                    posNeg = -1
            
            # elif key in angPositionalBindings:
            #     if key == 'w':
            #         deg = 1
            #     elif key == 'x':
            #         deg = 15

            elif key == 'm':
                break
            
            else:
                pass

    except Exception as e:
        print(e)