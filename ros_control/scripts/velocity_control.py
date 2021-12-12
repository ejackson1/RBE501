#!/usr/bin/python3

import rospy
from std_msgs.msg import Float64
from controller_manager_msgs.srv import SwitchController

if __name__ == "__main__":
    
     rospy.init_node("robot_velocity_control")
     pub1 = rospy.Publisher("/my_gen3/joint_1_velocity_controller/command", Float64, queue_size=10)
     pub2 = rospy.Publisher("/my_gen3/joint_2_velocity_controller/command", Float64, queue_size=10)
     pub3 = rospy.Publisher("/my_gen3/joint_3_velocity_controller/command", Float64, queue_size=10)
     pub4 = rospy.Publisher("/my_gen3/joint_4_velocity_controller/command", Float64, queue_size=10)
     pub5 = rospy.Publisher("/my_gen3/joint_5_velocity_controller/command", Float64, queue_size=10)
     pub6 = rospy.Publisher("/my_gen3/joint_6_velocity_controller/command", Float64, queue_size=10)
     pub7 = rospy.Publisher("/my_gen3/joint_7_velocity_controller/command", Float64, queue_size=10)

     rospy.sleep(1)

     rospy.wait_for_service('/my_gen3/controller_manager/switch_controller')
     try:
          sc_service = rospy.ServiceProxy('/my_gen3/controller_manager/switch_controller', SwitchController)
          start_controllers = ['joint_1_velocity_controller','joint_2_velocity_controller','joint_3_velocity_controller','joint_4_velocity_controller','joint_5_velocity_controller','joint_6_velocity_controller','joint_7_velocity_controller']
          stop_controllers = ['gen3_joint_trajectory_controller']
          strictness = 2
          start_asap = False
          timeout = 0.0
          res = sc_service(start_controllers,stop_controllers, strictness, start_asap,timeout)
          rospy.loginfo('switching successful')
     except rospy.ServiceException as e:
          rospy.loginfo("Service Call Failed")	

     pub1.publish(1)
     pub2.publish(1)
     pub3.publish(1)
     pub4.publish(1)
     pub5.publish(1)
     pub6.publish(1)
     pub7.publish(1)
