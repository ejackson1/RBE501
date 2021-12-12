#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_geometry_msgs
import tf2_ros

class jointTrajectoryClient:
    def __init__(self) -> None:
        rospy.loginfo("Initializing Joint Trajectory Node")
        rospy.init_node("jointTrajectoryClient", anonymous=True)

        self.traj_client = rospy.Publisher('my_gen3/gen3_joint_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.arm_names = ["joint_1", "joint_2", "joint_3", "joint_4",
         "joint_5", "joint_6", "joint_7"]

        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.loginfo("Ready!")

    
    def send_trajectories(self):
            print("sending trajectories!")
            goal_positions = [-1.211, 1.812, -3.825, 1.253,
                -6.39,  6.276, 1.95]
            arm_trajectory = JointTrajectory()
            arm_trajectory.joint_names = self.arm_names
            arm_trajectory.points.append(JointTrajectoryPoint())
            arm_trajectory.points[0].positions = goal_positions
            arm_trajectory.points[0].velocities = [0, 0, 0, 0, 0, 0, 0]
            arm_trajectory.points[0].accelerations = [0, 0, 0, 0, 0, 0, 0]
            arm_trajectory.points[0].time_from_start = rospy.Duration(0.2)
            rospy.sleep(1)
            self.traj_client.publish(arm_trajectory)

            rospy.sleep(8)
            try:
                self.pos = self.tfBuffer.lookup_transform('world', 'end_effector_link', rospy.Time())
                print(self.pos)
            except Exception as e:
                print(e)
                





if __name__ == "__main__":
    jointTrajClient = jointTrajectoryClient()
    jointTrajClient.send_trajectories()
    rospy.spin()

    

