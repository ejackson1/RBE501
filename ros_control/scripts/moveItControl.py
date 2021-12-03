#!/usr/bin/env python3

import tf
import rospy
import actionlib

from ros_control.msg import MoveArmAction, MoveArmGoal, MoveArmCartAction, MoveArmCartGoal
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    rospy.init_node("MoveItNode")

    move_client = actionlib.SimpleActionClient('move_arm', MoveArmAction)
    cart_client = actionlib.SimpleActionClient('move_arm_cart', MoveArmCartAction)

    print("a")
    move_client.wait_for_server()
    cart_client.wait_for_server()


    rospy.loginfo("Sending to coordinates...")
    approach = PoseStamped()

    approach.pose.position.x = 0.65
    approach.pose.position.y = 0.2
    approach.pose.position.z = 0.7


    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)

    approach.pose.orientation.x = quaternion[0]
    approach.pose.orientation.y = quaternion[1]
    approach.pose.orientation.z = quaternion[2]
    approach.pose.orientation.w = quaternion[3]

    goal = MoveArmGoal()

    goal.ee_pose = approach
    goal.ee_pose.header.frame_id = "world"
    goal.ee_pose.header.stamp = rospy.Time.now()


    goal.tolerance = 0.005

    move_client.send_goal_and_wait(goal)

    rospy.loginfo("Ta-da!")

