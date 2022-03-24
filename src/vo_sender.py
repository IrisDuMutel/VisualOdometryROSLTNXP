#!/usr/bin/env python3

# This node sends the data from topic /odometry/filtered through the MAVLink link 
# The odometry values are available to the FRDM to act appropriately

import rospy
import message_filters
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time
import rosbag
import bagpy
from bagpy import bagreader
import os
os.environ['MAVLINK20']='1' # set mavlink2 for odometry message
from pymavlink import mavutil

connection = mavutil.mavlink_connection('udpout:192.168.1.10:8150')


def mavlink_send(odom_sub):#, imu_sub):#,plan_sub):
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

    
  

def mavlink_manager():
    rospy.init_node('mavlink_manager', anonymous=True)
    odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, callback=mavlink_send, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    try:
        mavlink_manager()
    except rospy.ROSInterruptException:
        pass