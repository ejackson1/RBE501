#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_geometry_msgs
import tf2_ros
import copy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from ros_control.srv import jointTraj, jointTrajResponse

class jointTrajectoryClient:
    def __init__(self) -> None:
        rospy.loginfo("Initializing Joint Trajectory Node")
        rospy.init_node("jointTrajectoryClient", anonymous=True)

        self.traj_client = rospy.Publisher('my_gen3/gen3_joint_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.arm_names = ["joint_1", "joint_2", "joint_3", "joint_4",
         "joint_5", "joint_6", "joint_7"]
        self.motion_started = 0
        # rospy.Subscriber("/my_gen3/joint_states", JointState, self.joint_callback)

        s = rospy.Service('/quintic', jointTraj, self.send_trajectories)
        s1 = rospy.Service('/position', jointTraj, self.send_positions)
        
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        
        

        self.q1List = []
        self.q2List = []
        self.q3List = []
        self.q4List = []
        self.q5List = []
        self.q6List = []
        self.q7List = []
        self.qd1List = []
        self.qd2List = []
        self.qd3List = []
        self.qd4List = []
        self.qd5List = []
        self.qd6List = []
        self.qd7List = []
        self.timeListplot = []

        rospy.loginfo("Ready!")

    # def joint_callback(self, msg):
    #     if self.motion_started == 1:
            
            
    #         self.q1List.append(msg.position[0])
    #         self.q2List.append(msg.position[1])
    #         self.q3List.append(msg.position[2])
    #         self.q4List.append(msg.position[3])
    #         self.q5List.append(msg.position[4])
    #         self.q6List.append(msg.position[5])
    #         self.q7List.append(msg.position[6])
    #         self.qd1List.append(msg.velocity[0])
    #         self.qd2List.append(msg.velocity[1])
    #         self.qd3List.append(msg.velocity[2])
    #         self.qd4List.append(msg.velocity[3])
    #         self.qd5List.append(msg.velocity[4])
    #         self.qd6List.append(msg.velocity[5])
    #         self.qd7List.append(msg.velocity[6])
    #         self.curr_time = rospy.Time.now()
    #         self.curr_time = (self.curr_time.secs + self.curr_time.nsecs * 10**-9)
    #         self.timeListplot.append(self.curr_time)

            
        
    #     elif self.motion_started == 2:
    #         print("q1List " + str(self.q1List))
    #         plt.figure(1)
    #         plt.subplot(711)
    #         plt.plot(self.timeListplot  ,self.q1List)
    #         plt.title("Q1")

    #         plt.subplot(712)
    #         plt.plot(self.timeListplot, self.q2List)
    #         plt.title("Q2")

    #         plt.subplot(713)
    #         plt.plot(self.timeListplot, self.q3List)
    #         plt.title("Q3")

    #         plt.subplot(714)
    #         plt.plot(self.timeListplot, self.q4List)
    #         plt.title("Q4")

    #         plt.subplot(715)
    #         plt.plot(self.timeListplot, self.q5List)
    #         plt.title("Q5")

    #         plt.subplot(716)
    #         plt.plot(self.timeListplot, self.q6List)
    #         plt.title("Q6")

    #         plt.subplot(717)
    #         plt.plot(self.timeListplot, self.q7List)
    #         plt.title("Q7")
    #         plt.show()


    #         plt.figure(1)
    #         plt.subplot(711)
    #         plt.plot(self.timeListplot  ,self.qd1List)
    #         plt.title("Qd1")

    #         plt.subplot(712)
    #         plt.plot(self.timeListplot, self.qd2List)
    #         plt.title("Qd2")

    #         plt.subplot(713)
    #         plt.plot(self.timeListplot, self.qd3List)
    #         plt.title("Qd3")

    #         plt.subplot(714)
    #         plt.plot(self.timeListplot, self.qd4List)
    #         plt.title("Qd4")

    #         plt.subplot(715)
    #         plt.plot(self.timeListplot, self.qd5List)
    #         plt.title("Qd5")

    #         plt.subplot(716)
    #         plt.plot(self.timeListplot, self.qd6List)
    #         plt.title("Qd6")

    #         plt.subplot(717)
    #         plt.plot(self.timeListplot, self.qd7List)
    #         plt.title("Qd7")
    #         plt.show()
    #         self.motion_started = 0
    #     pass

    def send_positions(self, msg):
        print("Sending Positions!")
        print(msg)
        goal_positions = [msg.q1[0], msg.q2[0], msg.q3[0], msg.q4[0],
                         msg.q5[0], msg.q6[0], msg.q7[0]]
        print(goal_positions)
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = self.arm_names
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = goal_positions
        arm_trajectory.points[0].velocities = [0, 0, 0, 0, 0, 0, 0]
        arm_trajectory.points[0].accelerations = [0, 0, 0, 0, 0, 0, 0]
        arm_trajectory.points[0].time_from_start = rospy.Duration(msg.times[0])
        bool =Bool()
        # try:
        #     self.traj_client.publish(arm_trajectory)

        #     bool.data = True
        #     return jointTrajResponse(bool)
        
        # except Exception as e:
        #     print(e)
        #     bool.data = False
        #     return jointTrajResponse(bool)


        try:
            print(arm_trajectory)
            #self.time_start = rospy.Time.now()
            #self.motion_started = 1
            self.traj_client.publish(arm_trajectory)
            #rospy.sleep(1.5)
            #self.motion_started = 2
            #self.time_finished = rospy.Time.now()
            #self.secDuration = (self.time_finished.secs + self.time_finished.nsecs * 10**-9) - (self.time_finished.secs + self.time_finished.nsecs * 10**-9)

            
            bool.data = True
            return jointTrajResponse(bool)
        
        except Exception as e:
            print(e)
            bool.data = False
            return jointTrajResponse(bool)
        

    def send_trajectories(self, msg):
        print("Sending trajectories!")
        #print(msg)
        ## Parsing Message
        goal_positions = []
        goal_velocities = []
        goal_accelerations = []
        for (q1, q2, q3, q4, q5, q6, q7, 
            qd1, qd2, qd3, qd4, qd5, qd6, qd7, 
            qdd1, qdd2, qdd3, qdd4, qdd5, qdd6, qdd7) in  zip(msg.q1, msg.q2, msg.q3, msg.q4, msg.q5, msg.q6, msg.q7,
                                                            msg.qd1, msg.qd2, msg.qd3, msg.qd4, msg.qd5, msg.qd6, msg.qd7,
                                                            msg.qdd1, msg.qdd2, msg.qdd3, msg.qdd4, msg.qdd5, msg.qdd6, msg.qdd7):
            goal_positions.append([q1, q2, q3, q4, q5, q6, q7])
            goal_velocities.append([qd1, qd2, qd3, qd4, qd5, qd6, qd7])
            goal_accelerations.append([qdd1, qdd2, qdd3, qdd4, qdd5, qdd6, qdd7])

        
        #print(goal_positions)
        #print(msg.times)
        

        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = self.arm_names

        jointTrajPt = JointTrajectoryPoint()
        for (goal, vel, acc, time) in zip(goal_positions, goal_velocities, goal_accelerations, msg.times):
            #print(time)
            jointTrajPt.positions = goal
            jointTrajPt.velocities = vel
            jointTrajPt.accelerations = acc
            #print(rospy.Duration(time))
            jointTrajPt.time_from_start = rospy.Duration(time)
            
            arm_trajectory.points.append(copy.deepcopy(jointTrajPt))
        bool = Bool()
        try:
            print(arm_trajectory)
            self.time_start = rospy.Time.now()
            #self.motion_started = 1
            self.traj_client.publish(arm_trajectory)
            #rospy.sleep(1.5)
            #self.motion_started = 2
            #self.time_finished = rospy.Time.now()
            #self.secDuration = (self.time_finished.secs + self.time_finished.nsecs * 10**-9) - (self.time_finished.secs + self.time_finished.nsecs * 10**-9)

            
            bool.data = True
            return jointTrajResponse(bool)
        
        except Exception as e:
            print(e)
            bool.data = False
            return jointTrajResponse(bool)


            #self.traj_client.publish(arm_trajectory)
            #jointTrajPt



            # goal_positions = [-1.211, 1.812, -3.825, 1.253,
            #     -6.39,  6.276, 1.95]
            # arm_trajectory = JointTrajectory()
            # arm_trajectory.joint_names = self.arm_names
            # arm_trajectory.points.append(JointTrajectoryPoint())
            # arm_trajectory.points[0].positions = goal_positions
            # arm_trajectory.points[0].velocities = [0, 0, 0, 0, 0, 0, 0]
            # arm_trajectory.points[0].accelerations = [0, 0, 0, 0, 0, 0, 0]
            # arm_trajectory.points[0].time_from_start = rospy.Duration(1)
            # #rospy.sleep(1)
            # #self.traj_client.publish(arm_trajectory)

            # #print("sending trajectories!")
            # goal_positions = [0, 0, 0, 0,
            #     0, 0, 0]
            # #arm_trajectory = JointTrajectory()
            # #arm_trajectory.joint_names = self.arm_names
            # #arm_trajectory.points.append(JointTrajectoryPoint())
            # joint_Traj = JointTrajectoryPoint()
            # joint_Traj.positions = goal_positions
            # joint_Traj.velocities = [0, 0, 0, 0, 0, 0, 0]
            # joint_Traj.accelerations = [0, 0, 0, 0, 0, 0, 0] 
            # joint_Traj.time_from_start = rospy.Duration(2)
            # arm_trajectory.points.append(joint_Traj)
            # print(arm_trajectory)
            # rospy.sleep(1)
            # self.traj_client.publish(arm_trajectory)

            

            

            # rospy.sleep(8)
            # try:
            #     self.pos = self.tfBuffer.lookup_transform('world', 'end_effector_link', rospy.Time())
            #     print(self.pos)
            # except Exception as e:
            #     print(e)
                





if __name__ == "__main__":
    jointTrajClient = jointTrajectoryClient()
    #jointTrajClient.send_trajectories()


    rospy.spin()

    

