#!/usr/bin/env python3

# This code si a simulation of what data_fused_receiver.py would do if the mavlink link was up
# It is useful to use it to see how the EKF behaves


from email import message
from math import cos, sin, sqrt,pi
import queue
import rospy
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from visual_odometry.msg import PWM_cmd
import time
import os
from scipy.spatial.transform import Rotation as R




class APF():
    
    def __init__(self):
        rospy.init_node('APF',anonymous=True)
        
        sub_destination = message_filters.Subscriber("/goal", Pose, queue_size = 10)
        sub_odometry    = message_filters.Subscriber("/odometry/filtered", Odometry, queue_size = 10)
        # print('AAAAAAAAAAAAAAAAAAAA')

        self.msg = PWM_cmd()
        ts = message_filters.ApproximateTimeSynchronizer([sub_destination,sub_odometry], queue_size=10, slop=0.5, allow_headerless=True)
        ts.registerCallback(self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        
    def callback(self,sub_destination, sub_odometry):
        # rospy.loginfo('AAAAAAAAAAAAAAAAAAAA')
        

if __name__ == '__main__':
    try:
        APF()
    except rospy.ROSInterruptException:
        pass