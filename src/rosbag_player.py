#!/usr/bin/env python3

# This code si a simulation of what data_fused_receiver.py would do if the mavlink link was up
# It is useful to use it to see how the EKF behaves


from math import cos, sin, sqrt,pi
from unittest.mock import call

from click import command
import rospy
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Imu
from visual_odometry.msg import APF_cmd
import time
import os
from scipy.spatial.transform import Rotation as R
# os.environ['MAVLINK20']='1' # set mavlink2 for odometry message
# from pymavlink import mavutil
# K64F_IP = '192.168.1.10' # Freedom K64F IP Address
# K64F_PORT='8150'         # Freedom K64F UPD port
# ADDRESS = 'upd:'+ K64F_IP + ':' + K64F_PORT
# connection_in = mavutil.mavlink_connection('udp:0.0.0.0:8151', input=True)
class Rosbag_player():
    def __init__(self):
        rospy.init_node('rosbag_player_command', anonymous=True)
        # rate = rospy.Rate(10) # 10hz
        # self.pub_enc = rospy.Publisher('odom', Odometry, queue_size=10)
        # self.sub_mix = rospy.Subscriber('odometry/filtered',Odometry,self.callback)
        self.pub_enc = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub_mix = rospy.Subscriber('APF_output',APF_cmd,self.callback)
        self.psi_old = 0
        self.curr_time =  rospy.get_rostime()
        self.command_vel = Twist()
        rospy.spin()
        # pub_mag = rospy.Publisher('magnetometer', Odometry,queue_size=10)
        # pub_imu = rospy.Publisher('imu_k64', Imu, queue_size=10)
        # pub_imu2 = rospy.Publisher('imu_ext', Imu, queue_size=10)


    def callback(self,data):
        # forced_odom = Odometry()
        # forced_odom = data
        x_vel = data.Vref
        psi_vel = data.Psiref
        elapsed_time = rospy.get_rostime().secs-self.curr_time.secs
        self.curr_time = rospy.get_rostime().secs
        
        self.command_vel.linear.x = x_vel
        self.command_vel.angular.z = (psi_vel-self.psi_old)/elapsed_time
        self.psi_old = psi_vel
        
        self.pub_enc.publish(self.command_vel)
        
        
        
        
    
if __name__ == '__main__':
    try:
        
        Rosbag_player()
    except rospy.ROSInterruptException:
        pass