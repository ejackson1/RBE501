#!/usr/bin/env python3

import sys
import rospy
import actionlib
import moveit_commander

from ros_control.msg import MoveArmAction, MoveArmResult, MoveArmCartAction, MoveArmCartResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

joint_states = JointState()

# Tolerance for EE pose
GOAL_TOLERANCE = 0.001

# Action server that handles requests to send goals to MoveIt and relay waypoints to cFS
class MoveArmServer:
  def __init__(self, name):

    rospy.loginfo("Initializing action server and commanders...")
    self.move_server = actionlib.SimpleActionServer("move_arm", MoveArmAction, execute_cb=self.move_cb, auto_start=False)
    self.move_server.start()

    self.cart_server = actionlib.SimpleActionServer("move_arm_cart", MoveArmCartAction, execute_cb=self.cart_cb, auto_start=False)
    self.cart_server.start()

    #self.robot = moveit_commander.RobotCommander(robot_description="my_gen3/robot_description")

    #self.arm_group = moveit_commander.MoveGroupCommander("my_gen3_macro")
    #self.planning_scene = moveit_commander.PlanningSceneInterface()

    # Create the MoveItInterface necessary objects
    arm_group_name = "arm"
    self.robot = moveit_commander.RobotCommander("robot_description")
    self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
    self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
    #self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
    #                                            moveit_msgs.msg.DisplayTrajectory,
    #                                            queue_size=20)

    self.js_sub = rospy.Subscriber("/my_gen3/base_feedback/joint_states", JointState, self.js_cb)
    
    rospy.sleep(2)

    rospy.loginfo("Connected to MoveGroup and Robot Commanders.")

  def js_cb(self, js):
    global joint_states
    joint_states = js

  def move_cb(self, goal):
    rospy.loginfo("Received move goal! Getting plan...")

    self.setGripper(goal.open)

    self.arm_group.set_pose_target(goal.ee_pose)
    self.arm_group.set_pose_reference_frame(goal.ee_pose.header.frame_id)
    self.arm_group.set_goal_position_tolerance(goal.tolerance)
      
    rospy.loginfo("Executing...")
    plan = self.arm_group.go(wait=True)

    self.arm_group.stop()
    success = True

    if(success):
      rospy.loginfo("Plan from move group successful.")
    else:
      rospy.logerr("Plan failed, most likely cause is the goal is outside the task space.")
      self.server.set_aborted(MoveArmResult(success))
      return

    # Once the trajectory has finished, succeed on the AS goal
    self.move_server.set_succeeded(MoveArmResult(success))

  def cart_cb(self, goal):
    rospy.loginfo("Received cartesian goal! Getting plan...")

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, _) = self.arm_group.compute_cartesian_path(
                                      goal.ee_poses,   # waypoints to follow
                                      0.01,            # eef_step
                                      0.0)             # jump_threshold

    self.arm_group.execute(plan, wait=True)

    # Once the trajectory has finished, succeed on the AS goal
    self.cart_server.set_succeeded(MoveArmResult(True))


if __name__ == "__main__":
  rospy.init_node("moveit_interface")
  args = rospy.myargv(argv=sys.argv)

  moveit_commander.roscpp_initialize(sys.argv)

  server = MoveArmServer(rospy.get_name())

  rospy.spin()