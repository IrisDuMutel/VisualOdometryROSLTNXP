#!/usr/bin/env python3

# This node sends the data from topic /odometry/filtered through the MAVLink link 
# The odometry values are available to the FRDM to act appropriately

import rospy
import message_filters
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from visual_odometry.msg import PWM_cmd
import time
import rosbag
import bagpy
from bagpy import bagreader
import numpy as np
import os
os.environ['MAVLINK20']='1' # set mavlink2 for odometry message
from pymavlink import mavutil
# K64F_IP = '192.168.1.10' # Freedom K64F IP Address
# K64F_PORT='8150'         # Freedom K64F UPD port
# ADDRESS = 'upd:'+ K64F_IP + ':' + K64F_PORT
connection = mavutil.mavlink_connection('udpout:192.168.1.10:8150')
# connection_out = mavutil.mavlink_connection('udpin:192.168.1.10:8151')

# odom_bag = rosbag.Bag('~/Desktop/odom.bag', 'w')

def mavlink_send(odom_sub,PID_sub):#, imu_sub):#,plan_sub):
    ### POSE DATA
    # Preparing odometry mavlink message
    time_usec = 0 
    frame_id = 0
    child_frame_id = 0
    # Position
    x = odom_sub.pose.pose.position.x
    y = odom_sub.pose.pose.position.y
    z = odom_sub.pose.pose.position.z
    # Quaternion
    q = [odom_sub.pose.pose.orientation.w,odom_sub.pose.pose.orientation.x,odom_sub.pose.pose.orientation.y,odom_sub.pose.pose.orientation.z]
    # Linear velocity
    vx = odom_sub.twist.twist.linear.x
    vy = odom_sub.twist.twist.linear.y
    vz = odom_sub.twist.twist.linear.z
    # Angular velocity
    rollspeed = odom_sub.twist.twist.angular.x
    pitchspeed = odom_sub.twist.twist.angular.y
    yawspeed = odom_sub.twist.twist.angular.z
    # The following fields are not used
    pose_covariance = []
    velocity_covariance = []
    
    # PWM command
    PWM_right = PID_sub.PWM_right
    PWM_left  = PID_sub.PWM_left
    
    for i in range(21):
        pose_covariance.append(0)
        velocity_covariance.append(0)
    reset_counter = 0
    estimator_type = 0
    # Send odometry mavlink message
    connection.mav.odometry_send(time_usec,\
                                frame_id,\
                                child_frame_id,\
                                x,y,z,\
                                q,\
                                vx,vy,vz,\
                                rollspeed,pitchspeed,yawspeed,\
                                pose_covariance,velocity_covariance,\
                                reset_counter,estimator_type)

    connection.mav.rc_channels_send(0, 2, int(np.abs(PWM_left)), int(np.abs(PWM_right)), int(np.sign(PWM_left) + 2), int(np.sign(PWM_right)+2), 0, 0,0,0,0,0,0,0,0,0,0,0,0,0,254)
        # time.sleep(0.2)
    
  

def mavlink_manager():
    rospy.init_node('mavlink_manager', anonymous=True)
    odom_sub = message_filters.Subscriber("/odometry/filtered", Odometry, queue_size=10)
    PID_sub = message_filters.Subscriber("/PID_cmd", PWM_cmd, queue_size = 10)
    # imu_sub =rospy.Subscriber("/imu/data", Imu, callback=mavlink_send, queue_size=1)
    ts = message_filters.ApproximateTimeSynchronizer([odom_sub,PID_sub], queue_size=10, slop=0.5, allow_headerless=True)
    ts.registerCallback(mavlink_send)
    rospy.spin()


if __name__ == '__main__':
    try:
        mavlink_manager()
    except rospy.ROSInterruptException:
        pass