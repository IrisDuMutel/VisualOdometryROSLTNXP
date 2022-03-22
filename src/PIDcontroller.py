#!/usr/bin/env python3

# This is the code from the PIDcontroller Enza did on MATLAB but translated to Pyhton by Iris

from numpy import linalg as LA
import math
import queue

from zmq import EVENT_CLOSE_FAILED
import rospy
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from visual_odometry.msg import PWM_cmd,APF_cmd
from scipy.spatial.transform import Rotation as R


class PIDcontroller():
    
    def __init__(self):
        rospy.init_node('PIDcontroller',anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
        sub_APFout = message_filters.Subscriber("/APF_output", APF_cmd, queue_size = 10)
        sub_goal = message_filters.Subscriber("/goal", Pose, queue_size = 10)
        sub_odometry    = message_filters.Subscriber("/odometry/filtered", Odometry, queue_size = 10)
        self.PIDpub = rospy.Publisher('PID_cmd', PWM_cmd, queue_size=10)
        self.msg = PWM_cmd()
        ts = message_filters.ApproximateTimeSynchronizer([sub_APFout,sub_odometry,sub_goal], queue_size=10, slop=0.5, allow_headerless=True)
        ts.registerCallback(self.callback)
        
        ## PID Control parameters 
        self.Kp_v = 1
        self.Ki_v = 0
        self.Kd_v = 0
        self.Kp_psi = 1
        self.Ki_psi = 0    
        self.Kd_psi = 0    
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        
    def get_rotation(self,Odom):
        orientation_q = Odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        r = R.from_quat(orientation_list)
        EuAn = r.as_euler('zyx', degrees=False)
        return EuAn 
        
    def callback(self,sub_APFout, sub_odometry, sub_goal):
        # print('BBBBBBBBBBBBBBBBBBBBBB')
        
        V_ref = sub_APFout.Vref
        Psi_ref = sub_APFout.Psiref
        
        [yaw_ekf, _, _] = self.get_rotation(sub_odometry)
        
        v_x = sub_odometry.twist.twist.linear.x
        v_y = sub_odometry.twist.twist.linear.y
        x_est = sub_odometry.pose.pose.position.x
        y_est = sub_odometry.pose.pose.position.y
        x_goal = sub_goal.position.x
        y_goal = sub_goal.position.y
        Pos_est = [x_est,y_est]
        Pos_ref = [x_goal,y_goal]
        Psi_est = yaw_ekf
        Psidot_ref = 0
        Psidot_est = sub_odometry.twist.twist.angular.z
        V_est = (v_x+v_y)/(math.cos(Psi_est)+math.sin(Psi_est))
        
        ## X-V control
        X_error = math.dist(Pos_est,Pos_ref)
        V_error = V_ref- V_est
        V_cmd = self.Kp_v*V_error + self.Ki_v*X_error
        
        
        ## Psi control
        Psi_ref = Psi_ref*180/math.pi
        Psi_est = Psi_est*180/math.pi
        
        Psi_error=Psi_ref-Psi_est
        Psidot_error = Psidot_est-Psidot_ref
        if abs(Psi_error)>180:
            Psi_error=Psi_error-360*math.copysign(1, Psi_error)
        
        Psi_cmd = 1/180*(self.Kp_psi*Psi_error+self.Kd_psi*Psidot_error)
        
        ## Saturation
        if V_cmd > 1:
            V_cmd = 1
        elif V_cmd < -1:
            V_cmd = -1
        
        if Psi_cmd > 0.5:
            Psi_cmd = 0.5
        elif Psi_cmd < -0.5:
            Psi_cmd = -0.5
        
        ## Command module: transforms V and Psi cmds into PWM cmds
        
        PWM_right = (V_cmd+Psi_cmd)*20000
        PWM_left  = (V_cmd-Psi_cmd)*20000
        
        ## Saturation
        if PWM_right > 20000:
            PWM_right = 20000
        elif PWM_right < -20000:
            PWM_right = -20000
        
        if PWM_left > 20000:
            PWM_left = 20000
        elif PWM_left < -20000:
            PWM_left = -20000
        
        self.msg.PWM_right = PWM_right
        self.msg.PWM_left  = PWM_left
        
        self.PIDpub.publish(self.msg)

if __name__ == '__main__':
    try:
        PIDcontroller()
    except rospy.ROSInterruptException:
        pass