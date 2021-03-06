#!/usr/bin/env python
import rospy
# from jetbotcar.msg import Jetdrivemsg # float64 left, right
# from jetbotcar.msg import jetRacerDriveMsg # float64 throttle, steering
# from jetbotcar.msg import jetErrorMsg # float64 lateralError
from geometry_msgs.msg import Twist, Pose
# from scipy import integrate
 
import rospy
import time
 
from std_msgs.msg import Float64, String
 
######
import sys
import math
import numpy as np
 
 

#PID CONTROL PARAMS
kp = 3.0 #0.005 
kd = 0.115#0.0 
ki = 0.125 #0.0 
 
lastError = 0.0
publish = bool(False)
 

# INITIALISE THE THROTTLE AND STEERING Cmds
throttleCmd = 0.0
steeringCmd = 0.0
steeringGain = 0.0
windupMax = 2.0
cumError = 0.0
pidOutput=0.0
 
def startCallback(msg):
    global publish

    if msg.data == "i":
        publish = bool(True)
        # rospy.loginfo("publish, %s", publish)
 

def pidCallback(msg):
 
    # global startTime
    global previousTime
    global currentTime
    global cumError
    global lastError
    global pidOutput
 
    global kp, ki, kd
    global throttleCmd, steeringCmd
    global publish
 
    
 
    lateralErrorCmd = msg.data # assign error messages from image processing to lateralError
    # Time stamp
 
    currentTime = rospy.get_time()
    elapsedTime = currentTime - previousTime

    # Compute all the working error variables, careful with the sign convension
    error = lateralErrorCmd #lateral error is the error input from image processing
    cumError = cumError + error * elapsedTime
    # cumError = integrate.quad(error, 0, 0.1)
    rateError = (error - lastError)/elapsedTime
 
    #  anti-windup for integral item
    if cumError > windupMax:
        cumError = windupMax
    elif cumError < - windupMax:
        cumError = windupMax

    # PID output computation
    pidOutput = (kp * error + ki * cumError + kd * rateError) /1000.0 # need satruation
 

    # rospy.loginfo("Current time: %.2f", currentTime)
    # rospy.loginfo("Elapsed time: %.2f", elapsedTime)
    # rospy.loginfo("lateral error: %.2f", error)
    # rospy.loginfo("Cumulative Error: %.2f", cumError)
    # rospy.loginfo("rate error: %.2f", rateError)
    # rospy.loginfo("PID pidOutput: %.2f\n", pidOutput)
 

    # Remember some variables for next time
    lastError = error
    previousTime = currentTime
    throttleCmd = 1.4
    steeringCmd = pidOutput
 

    # if publish:
    rospy.loginfo("publish: %s", publish)
    publishCmdGazebo(throttleCmd, steeringCmd)
 
        # rospy.loginfo("throttle: %.2f", throttleCmd)
        # rospy.loginfo("steering: %.2f\n", steeringCmd)
 

def stopCallback(msg):
    global throttleCmd, steeringCmd
    global publish 
    if msg.data == "n":
        publish = bool(False)
        rospy.loginfo("publish: %s", publish)
 
        throttleCmd = 0.0
        steeringCmd = 0.0
        publishCmdGazebo(throttleCmd, steeringCmd)
        rospy.loginfo("vehicle has been stopped")
        rospy.loginfo("throttle: %.2f", throttleCmd)
        rospy.loginfo("steering: %.2f\n", steeringCmd)
 

def publishCmdGazebo(throttleCmd, steeringCmd): # publish function
    global drive_pub
 
    drive_msg = Twist()
    drive_msg.linear.x = throttleCmd
    drive_msg.angular.z = -steeringCmd
    drive_pub.publish(drive_msg)
 
def main():
    global throttleCmd, steeringCmd
    global currentTime
    global previousTime
    global drive_pub
    global pidOutput
    # initialise pid_controller node
    rospy.init_node("pid_controller", anonymous=True)
    previousTime = rospy.get_time() # startTime is constant now, stamped straight after init_node
 
    # subscribes to the lateral_error topic
 
    # *ADD* create the subscriber to the keyboard node
    # keyTopic = rospy.get_param("/jetRacerDriveNode/keyboard_topic")
    # rospy.Subscriber(keyTopic, String, stopCallback)
    # rospy.Subscriber(keyTopic, String, startCallback)
 
    # # Get topic name from the yaml file 
    # (just a topic title, doesn't include anything)
    # driveTopic = rospy.get_param("/cmd_vel")
 
    # jetRacerDriveMsg contains throttle and steering cmds
    drive_pub = rospy.Publisher("/cmd_vel",Twist, queue_size=10)
    rospy.Subscriber("/lateral_error", Float64, pidCallback)
    rate = rospy.Rate(100)
    pid_pub=rospy.Publisher("/pidoutput",Float64, queue_size=10)
    
    while not rospy.is_shutdown():
       pid_pub.publish(pidOutput)
       rate.sleep()
    #     # # Write a publishing function
    #     # drive_msg = jetRacerDriveMsg()
 
    #     # drive_msg.throttle = throttleCmd
    #     # drive_msg.steering = steeringCmd
 
    #     # # drive_pub.publish(drive_msg) # from now don't publish drive_msg to LLC
 
    #     publishCmdJetracer(throttleCmd, steeringCmd)
 
    #     # print("Publishing control commands for throttle and steering")
        
 
    publishCmdGazebo(throttleCmd, steeringCmd)
    rospy.spin()
 
if __name__=='__main__':
    main()
    # try:
    #     main()
    # except KeyboardInterrupt:
    #     print('Interrupted')
    #     try:
    #         sys.exit(0)
    #     except SystemExit:
    #         os._exit(0)
