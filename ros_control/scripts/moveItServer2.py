#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg

from ros_control.srv import moveToPose,moveToPoseResponse
from std_msgs.msg import Bool

class MoveItPlanner:
    def __init__(self) -> None:

        rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_interface", anonymous=True)
        
        s = rospy.Service('/move_it_planner', moveToPose, self.move_arm)

        self.robot = moveit_commander.RobotCommander("robot_description")
        self.arm_group_name = "arm"
        self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
        self.arm_group = moveit_commander.MoveGroupCommander(self.arm_group_name, ns=rospy.get_namespace())
        selfdisplay_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)
        
        self.TOLERANCE = 0.01
        
        rospy.loginfo("Ready to accept poses!")

    def get_cartesian_pose(self, arm_group):

        # Get the current pose and display it
        pose = arm_group.get_current_pose()
        rospy.loginfo("Actual cartesian pose is : ")
        rospy.loginfo(pose.pose)

        return pose.pose

    def move_arm(self, pose):
        bool = Bool()

        try:
            previous_pose = self.arm_group.get_current_pose()
            previous_pose = previous_pose.pose
            
            self.arm_group.set_goal_position_tolerance(self.TOLERANCE)
            self.arm_group.set_pose_target(pose.pose)
            rospy.loginfo("Planning and going to waypoint")
            self.arm_group.go(wait=True)

            current_pose = moveitplanner.get_cartesian_pose(self.arm_group)
            x = abs(previous_pose.position.x - current_pose.position.x)
            y = abs(previous_pose.position.y - current_pose.position.y)
            z = abs(previous_pose.position.z - current_pose.position.z)
            qx = abs(previous_pose.orientation.x - current_pose.orientation.x)
            qy = abs(previous_pose.orientation.y - current_pose.orientation.y)
            qz = abs(previous_pose.orientation.z - current_pose.orientation.z)
            qw = abs(previous_pose.orientation.w - current_pose.orientation.w)
            difference = []
            difference.extend([x,y,z,qx,qy,qz,qw])

            for value in difference:
                if value < 0.05:            
                    bool.data = False
                    return moveToPoseResponse(bool)
                else:
                    pass

            bool.data = True
            print("Success")
            return moveToPoseResponse(bool)

        except Exception as e:
            print(e)

            bool.data = False
            return moveToPoseResponse(bool)




if __name__ == "__main__":
    moveitplanner = MoveItPlanner()
    rospy.spin()
