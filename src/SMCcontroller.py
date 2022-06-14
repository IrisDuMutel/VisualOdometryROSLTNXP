#py_controll.py


from numpy import linalg as LA
import numpy as np
import math
import queue
import rospy
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from visual_odometry.msg import PWM_cmd,APF_cmd
from scipy.spatial.transform import Rotation as R



class SMC():


    def __init__(self):
       rospy.init_node('SMC',anonymous=True)
       self.rate = rospy.Rate(10) # 10hz
       sub_APFout = message_filters.Subscriber("/APF_output", APF_cmd, queue_size = 10)
       sub_goal = message_filters.Subscriber("/goal", Pose, queue_size = 10)
       sub_odometry    = message_filters.Subscriber("/odometry/filtered", Odometry, queue_size = 10)
       self.PIDpub = rospy.Publisher('SMC_cmd', PWM_cmd, queue_size=10)
       self.msg = PWM_cmd()
       self.psid_prec = 0
       self.vd_prec = 0
       ts = message_filters.ApproximateTimeSynchronizer([sub_APFout,sub_odometry,sub_goal], queue_size=10, slop=0.5, allow_headerless=True)
       ts.registerCallback(self.ctr_step)
       rospy.spin()

    def get_rotation(self,Odom):
        orientation_q = Odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        r = R.from_quat(orientation_list)
        EuAn = r.as_euler('zyx', degrees=False)
        return EuAn 


    def ctr_step(self,sub_APFout, sub_odometry, sub_goal):
        
        
        xd_dot = sub_odometry.twist.twist.linear.x
        v_y = sub_odometry.twist.twist.linear.y
        xr = sub_odometry.pose.pose.position.x
        yr = sub_odometry.pose.pose.position.y
        wr = sub_odometry.twist.twist.angular.z
        vd = sub_APFout.Vref

        psid = sub_APFout.Psiref
        wd = (psid-self.psid_prec)/0.1
        vd_dot = (vd-self.vd_prec)/0.1
        xd = xr + vd*0.1*math.cos(psid)
        yd = yr + vd*0.1*math.sin(psid)
        [psir, _, _] = self.get_rotation(sub_odometry)
        vr = (xd_dot+v_y)/(math.cos(psir)+math.sin(psir))

        
        
        
        #errors definition
        
        xe=math.cos(psir)*(xd-xr)+math.sin(psir)*(yd-yr)
        ye=-math.sin(psir)*(xd-xr)+math.cos(psir)*(yd-yr)
        psie=psid-psir
        
        if math.fabs(psie) > math.pi:
            psie=psie-2*math.pi*np.sign(psie)
        
        # smc - surfaces
        s1=xe
        s2=psie+math.atan(vd*ye)
    
        # smc - command evaluation
        eps1 = 0.3
        eps2 = 0.9
        #q1 = 1/(1+1/math.abs(s1))/math.exp(-s1)
        #q2 = 1/(1+1/math.abs(s2))/math.exp(-s2)
        u1=0.9*s1+0.7*s1/(np.abs(s1)+eps1)+wr*ye+vd*math.cos(psie)
        
        u2=(0.01*s2+0.5*s1/(np.abs(s2)+eps2)+wd+(ye*vd_dot+vd*vd*math.sin(psie))/(1+(ye*vd)*(ye*vd)))/(1+xe*vd/(1+(ye*vd)*(ye*vd)))
        
        if u1>0.45:
            u1 = 0.45

        if u1<0:
            u1=0

        if u2 > 0.1:
            u2 = 0.1

        if u2 < -0.1:
            u2 = -0.1
        
        PWM_R = 20000*(u1+u2)
        PWM_L = 20000*(u1-u2)

        if PWM_R>20000:
            PWM_R = 20000

        if PWM_R<-20000:
            PWM_R = -20000

        if PWM_L>20000:
            PWM_L = 20000

        if PWM_L<-20000:
            PWM_L = -20000

        self.psid_prec = psid
        self.vd_prec= vd

        self.msg.PWM_left = PWM_L
        self.msg.PWM_right = PWM_R
        self.PIDpub.publish(self.msg)


if __name__ == '__main__':
    try:
        SMC()
    except rospy.ROSInterruptException:
        pass