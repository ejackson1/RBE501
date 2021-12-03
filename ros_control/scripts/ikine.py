#!/usr/bin/env python3
import rospy
from math import degrees, atan2, sqrt, sin, cos
from for_tutorials.srv import Ik, IkResponse
from std_msgs.msg import Float64

THETA_1 = 0
THETA_2 = 0
DIST_3 = 0

def inverse_kinematics(position):
	global THETA_1, THETA_2, DIST_3
	D = (position.x**2 + position.y**2-2)/2
	theta2 = atan2(sqrt((1-D**2)), D)
	theta1 = atan2(position.y, position.x) - atan2(sin(theta2), (1+cos(theta2)))
	dist3 = position.z -1
	THETA_1, THETA_2, DIST_3 = theta1, theta2, dist3
	theta1 = degrees(theta1)
	theta2 = degrees(theta2)
	return [theta1,theta2,dist3]
    
def callback(req):
    return IkResponse(inverse_kinematics(req.end_effector_position))

if __name__ == '__main__':
	rospy.init_node('inverse_kinematics_node', anonymous=True)
	rospy.Service('inverse_kinematics', Ik, callback)
	pub_1 = rospy.Publisher("joint_1_position_controller/command",Float64, queue_size=10)
	pub_2 = rospy.Publisher("joint_2_position_controller/command",Float64, queue_size=10)
	pub_3 = rospy.Publisher("joint_3_position_controller/command",Float64, queue_size=10)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		pub_1.publish(THETA_1)
		pub_2.publish(THETA_2)
		pub_3.publish(DIST_3)
		rate.sleep()
