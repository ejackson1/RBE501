#!/usr/bin/env python3

import sys, select, termios, tty
import rospy
import geometry_msgs.msg
import math
from ros_control.srv import moveToPose
from rospy.core import rospyerr
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_geometry_msgs

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def degs2rads(degs):
    return degs / math.pi() 

def move_arm_client(pose):
    rospy.wait_for_service('/move_it_planner')
    try:
        c = rospy.ServiceProxy('/move_it_planner', moveToPose)
        success = c(pose)
        bool = Bool()
        bool.data = True
        if success.data == bool:
            return print(str(success.data.data) + ", we have a success")
        else:
            return print(str(success.data.data) + ", something went wrong")
    
    except rospy.ServiceException as e:
        print("Service called failed: %s"%e)

if __name__ == "__main__":

    msg = """
    Control Your Arm!
    ---------------------------
    Moving around:
    i j k : control direction
    r p y : control angular orientation
    t/b : control positive (w) or negative (s)

    q : adjust positional increments by 10cm
    e : adjust positional increments by 1 cm
    w : adjust angular position by 1deg
    r : adjust angular position by 15deg

    TODO 
    space key, k : force stop
    anything else : stop smoothly
    CTRL-C to quit
    """

    settings = termios.tcgetattr(sys.stdin)    
            
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = -0.5
    pose_goal.position.y = -3
    pose_goal.position.z = 0.3

    directionBindings = ['i', 'j', 'k']
    angularBindings = ['r', 'p', 'y']
    poseNegBindings = ['t', 'b']
    positionalBindings = ['q', 'e']
    angPositionalBindings = ['w', 'r']
    
    posNeg = 1
    direction = 'i'
    deg = 15

    try:
        print(msg) 
        while True:
            key = getKey()
            if key in directionBindings:
                # TODO
                if key == 'i':
                    # TODO
                    direction = 'i'
                elif key == 'j':
                    direction = 'j'
                else:
                    direction = 'k'

            elif key in positionalBindings:
                # TODO
                if key == 'q':
                    if direction == 'i':
                        # TODO
                        print("trying to move x")
                        pose_goal.position.x =+ (0.1 * posNeg)
                        move_arm_client(pose_goal)
                    elif direction == 'j':
                        pose_goal.position.y =+ (0.1 * posNeg)
                        move_arm_client(pose_goal)
                    elif direction == 'k':
                        pose_goal.position.z =+ (0.1 * posNeg)
                        move_arm_client(pose_goal)
                elif key == 'e':
                    if direction == 'i':
                        # TODO
                        pose_goal.position.x =+ (0.01 * posNeg)
                        move_arm_client(pose_goal)
                    elif direction == 'j':
                        pose_goal.position.y =+ (0.01 * posNeg)
                        move_arm_client(pose_goal)
                    elif direction == 'k':
                        pose_goal.position.z =+ (0.01 * posNeg)
                        move_arm_client(pose_goal)

            elif key in angPositionalBindings:
                if key == 'w':
                    deg = 1
                elif key == 'r':
                    deg = 15

            elif key in angularBindings:
                # TODO
                if key == 'r':
                    # TODO
                    if deg == 1:
                        (roll, pitch, yaw) = euler_from_quaternion(pose_goal.orientation)
                        roll += (posNeg * degs2rads(deg))
                        q = quaternion_from_euler(roll, pitch, yaw)
                        pose_goal.orientation.x = q[0]
                        pose_goal.orientation.y = q[1]
                        pose_goal.orientation.z = q[2]
                        pose_goal.orientation.w = q[3]
                        move_arm_client(pose_goal)
                    elif deg == 15:
                        (roll, pitch, yaw) = euler_from_quaternion(pose_goal.orientation)
                        roll += (posNeg * degs2rads(deg))
                        q = quaternion_from_euler(roll, pitch, yaw)
                        pose_goal.orientation.x = q[0]
                        pose_goal.orientation.y = q[1]
                        pose_goal.orientation.z = q[2]
                        pose_goal.orientation.w = q[3]
                        move_arm_client(pose_goal)

                elif key == 'p':
                    if deg == 1:
                        (roll, pitch, yaw) = euler_from_quaternion(pose_goal.orientation)
                        pitch += (posNeg * degs2rads(deg))
                        q = quaternion_from_euler(roll, pitch, yaw)
                        pose_goal.orientation.x = q[0]
                        pose_goal.orientation.y = q[1]
                        pose_goal.orientation.z = q[2]
                        pose_goal.orientation.w = q[3]
                        move_arm_client(pose_goal)
                    elif deg == 15:
                        (roll, pitch, yaw) = euler_from_quaternion(pose_goal.orientation)
                        pitch += (posNeg * degs2rads(deg))
                        q = quaternion_from_euler(roll, pitch, yaw)
                        pose_goal.orientation.x = q[0]
                        pose_goal.orientation.y = q[1]
                        pose_goal.orientation.z = q[2]
                        pose_goal.orientation.w = q[3]
                        move_arm_client(pose_goal)

                elif key == 'y':
                    if deg == 1:
                        (roll, pitch, yaw) = euler_from_quaternion(pose_goal.orientation)
                        yaw += (posNeg * degs2rads(deg))
                        q = quaternion_from_euler(roll, pitch, yaw)
                        pose_goal.orientation.x = q[0]
                        pose_goal.orientation.y = q[1]
                        pose_goal.orientation.z = q[2]
                        pose_goal.orientation.w = q[3]
                        move_arm_client(pose_goal)
                    elif deg == 15:
                        (roll, pitch, yaw) = euler_from_quaternion(pose_goal.orientation)
                        yaw += (posNeg * degs2rads(deg))
                        q = quaternion_from_euler(roll, pitch, yaw)
                        pose_goal.orientation.x = q[0]
                        pose_goal.orientation.y = q[1]
                        pose_goal.orientation.z = q[2]
                        pose_goal.orientation.w = q[3]
                        move_arm_client(pose_goal)


            elif key in poseNegBindings:
                if key == 't':
                    posNeg = 1
                elif key == 'b':
                    posNeg = -1

            elif key == 'm':
                break
            else:
                pass
        print("borken")

    # pose_goal.orientation.w = 1.0
    # pose_goal.position.x = -0.5
    # pose_goal.position.y = -3
    # pose_goal.position.z = 0.3
    # print("Requesting.. ")
    # move_arm_client(pose_goal)

    # pose_goal.orientation.w = 1.0
    # pose_goal.position.x = 0.4
    # pose_goal.position.y = -0.2
    # pose_goal.position.z = 0.4
    # print("Requesting.. ")
    # move_arm_client(pose_goal)

    # pose_goal.orientation.w = 1.0
    # pose_goal.position.x = -0.4
    # pose_goal.position.y = -0.2
    # pose_goal.position.z = 0.4
    # print("Requesting.. ")
    # move_arm_client(pose_goal)
    except Exception as e:
        print(e)
    
