#!/usr/bin/env python3

# This is the code from the PIDcontroller Enza did on MATLAB but translated to Pyhton by Iris

from numpy import linalg as LA
import math
import queue
import rospy
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from visual_odometry.msg import PWM_cmd,PIDcontroller_cmd

from scipy.spatial.transform import Rotation as R


class PIDcontroller():
    
    def __init__(self):
        rospy.init_node('PIDcontroller',anonymous=True)
        self.rate = rospy.Rate(10) # 10hz

        
    def callback(self,sub_destination, sub_obstacle, sub_odometry):



if __name__ == '__main__':
    try:
        PIDcontroller()
    except rospy.ROSInterruptException:
        pass