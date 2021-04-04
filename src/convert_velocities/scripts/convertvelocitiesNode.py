#!/usr/bin/env python
#rospy for the subscriber and publisher
#print(sys.version)
import rospy
# type of messages we are going to use:
# this is the command to the motors:
from std_msgs.msg import Float64
# this is the command of velocity from the keyboard node:
from geometry_msgs.msg import Twist, Pose
# Miscellaneous libraries that we might use:
import matplotlib.pyplot as plt
import numpy as np
import sys, getopt, math

back_left_wheel = 0
back_right_wheel = 0
front_left_hinge = 0
front_right_hinge = 0


def velocity_callback(msg):
    T = 0.15  #trackwidth of the rover
    L = 0.192
    rw = 0.034
    #Motors' velocities:
    global front_left_hinge, front_right_hinge, back_left_wheel, back_right_wheel
    x_dot = msg.linear.x
    back_left_wheel = x_dot / rw
    back_right_wheel = x_dot / rw
    front_left_hinge = msg.angular.z
    front_right_hinge = msg.angular.z
    


def main():
    global front_left_wheel, front_right_wheel, back_left_wheel, back_right_wheel
    rospy.init_node('VelocitiesConverter')
    rospy.Subscriber("/cmd_vel", Twist, velocity_callback)
    RWB_cmd_topic = "/rover/joint_back_right_wheel_velocity_controller/command"
    LWB_cmd_topic = "/rover/joint_back_left_wheel_velocity_controller/command"
    RWF_cmd_topic = "/rover/joint_front_right_hinge_position_controller/command"
    LWF_cmd_topic = "/rover/joint_front_left_hinge_position_controller/command"
    RWB_pub = rospy.Publisher(RWB_cmd_topic, Float64, queue_size=1)
    LWB_pub = rospy.Publisher(LWB_cmd_topic, Float64, queue_size=1)
    RWF_pub = rospy.Publisher(RWF_cmd_topic, Float64, queue_size=1)
    LWF_pub = rospy.Publisher(LWF_cmd_topic, Float64, queue_size=1)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        RWB_pub.publish(back_right_wheel)
        LWB_pub.publish(back_left_wheel)
        RWF_pub.publish(front_right_hinge)
        LWF_pub.publish(front_left_hinge)
        print("Converting keyboard commands to motor commands")
        rate.sleep()


if __name__ == '__main__':
    #pass
    main()
