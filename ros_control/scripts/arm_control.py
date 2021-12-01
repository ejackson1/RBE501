#!/usr/bin/python2

import rospy
from std_msgs.msg import Float64
from controller_manager_msgs.srv import SwitchController

if __name__ == "__main__":
    
    rospy.init_node("robot_control")
    pub1 = rospy.Publisher("/my_gen3/joint_1_position_controller/command", Float64, queue_size=10)
    pub2 = rospy.Publisher("/my_gen3/joint_2_position_controller/command", Float64, queue_size=10)
    pub3 = rospy.Publisher("/my_gen3/joint_3_position_controller/command", Float64, queue_size=10)
    pub4 = rospy.Publisher("/my_gen3/joint_4_position_controller/command", Float64, queue_size=10)
    pub5 = rospy.Publisher("/my_gen3/joint_5_position_controller/command", Float64, queue_size=10)
    pub6 = rospy.Publisher("/my_gen3/joint_6_position_controller/command", Float64, queue_size=10)
    pub7 = rospy.Publisher("/my_gen3/joint_7_position_controller/command", Float64, queue_size=10)
    rospy.sleep(1)
    
    rospy.wait_for_service('/my_gen3/controller_manager/switch_controller')
    try:
         sc_service = rospy.ServiceProxy('/my_gen3/controller_manager/switch_controller', SwitchController)
         start_controllers = ['joint_1_position_controller','joint_2_position_controller','joint_3_position_controller','joint_4_position_controller','joint_5_position_controller','joint_6_position_controller','joint_7_position_controller']
         stop_controllers = ['gen3_joint_trajectory_controller']
         strictness = 2
         start_asap = False
         timeout = 0.0
         res = sc_service(start_controllers,stop_controllers, strictness, start_asap,timeout)
         rospy.loginfo('switching successful')
    except rospy.ServiceException as e:
         rospy.loginfo("Service Call Failed")	

    msg = Float64()
    msg.data = 1.0 

    pub1.publish(msg)
    print("published")
