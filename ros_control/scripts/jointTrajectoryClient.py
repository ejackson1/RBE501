#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_geometry_msgs
import tf2_ros
import copy
from std_msgs.msg import Bool
from ros_control.srv import jointTraj, jointTrajResponse

class jointTrajectoryClient:
    def __init__(self) -> None:
        rospy.loginfo("Initializing Joint Trajectory Node")
        rospy.init_node("jointTrajectoryClient", anonymous=True)

        self.traj_client = rospy.Publisher('my_gen3/gen3_joint_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.arm_names = ["joint_1", "joint_2", "joint_3", "joint_4",
         "joint_5", "joint_6", "joint_7"]

        s = rospy.Service('/quintic', jointTraj, self.send_trajectories)
        s1 = rospy.Service('/position', jointTraj, self.send_positions)
        
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.loginfo("Ready!")

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
        try:
            self.traj_client.publish(arm_trajectory)

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
            self.traj_client.publish(arm_trajectory)

            
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

    

