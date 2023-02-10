#!/usr/bin/env python3
import numpy
import math
import rospy
from visual_odometry.msg import PWM_cmd
from visual_odometry.msg import Obsrv_cmd
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
class Obsrv():
    def __init__(self):
        rospy.init_node('SMO',anonymous=True)
        self.rate = rospy.Rate(100)
        sub_Enc = rospy.Subscriber('encoder',Odometry,self.Enc_callback)
        sub_Imu = rospy.Subscriber('imu_k64',Imu,self.Imu_callback)
        sub_Imue = rospy.Subscriber('imu_ext',Imu,self.Imue_callback)

        pub_Obs = rospy.Publisher('Obsrv_out',Obsrv_cmd,queue_size=10)
        self.Init()
        while not rospy.is_shutdown():
            self.Observer(pub_Obs)
            self.rate.sleep()

    def Observer(self,pub_Obs):
        f1 = self.areal*math.cos(self.psihat) -self.Vhat*self.w*math.sin(self.psihat)
        f2 = self.areal*math.sin(self.psihat) +self.Vhat*self.w*math.cos(self.psihat)
        a11 = 1.5
        a21 = 1.1
        a12 = 1.5
        a22 = 1.1 
        F1 = 1.2
        F2 = 1
        h1 = 1.5*math.sqrt(F2)
        h2 = 1.1*F2
        h3 = 1.5*math.sqrt(F2)
        h4 = 1.1*F2
        
        z11 = a11*math.sqrt(F1*abs(self.xreal-self.zhat1))*numpy.tanh(500*(self.xreal-self.zhat1))
        z21 = a21*F1*numpy.tanh(500*(self.xreal-self.zhat1))
        z12 = a12*math.sqrt(F1*abs(self.yreal-self.zhat2))*numpy.tanh(500*(self.yreal-self.zhat2))
        z22 = a22*F1*numpy.tanh(500*(self.yreal-self.zhat2))

        #zdhat1 = self.Vxhat + z11
        #zdhat2 = f1 + z21
        #zdhat3 = self.Vyhat + z12
        #zdhat4 = f2 + z22 
        
        zhat1_new = self.xhat + (self.Vxhat + z11)*0.01
        Vxhat_new = self.Vxhat + (f1 + z21)*0.01 
        zhat2_new = self.yhat + (self.Vyhat + z12)*0.01
        Vyhat_new = self.Vyhat + (f2 + z22)*0.01


        
        

        #Ce = self.Chat + h1*numpy.tanh(100*(self.xreal - self.xhat))
        xhat_new = self.xhat + 0.01*(self.Vhat*self.Chat + h1*math.sqrt(abs(self.xreal - self.xhat))*numpy.tanh(500*(self.xreal - self.xhat)))
        Chat_new = self.Chat + 0.01*(-self.w*self.Shat + h2*numpy.tanh(500*(self.xreal - self.xhat)))

        #Se = self.Shat + h3*numpy.tanh(100*(self.yreal - self.yhat))
        yhat_new = self.yhat + 0.01*(self.Vhat*self.Shat + h3*math.sqrt(abs(self.yreal - self.yhat))*numpy.tanh(500*(self.yreal - self.yhat)))
        Shat_new = self.Shat + 0.01*(self.w*self.Chat + h4*numpy.tanh(500*(self.yreal - self.yhat)))

        psihat = numpy.arctan2(Shat_new,Chat_new)
        Vhat = math.sqrt(Vxhat_new**2 + Vyhat_new**2)
        
        self.zhat1 = zhat1_new
        self.zhat2 = zhat2_new
        self.Vxhat = Vxhat_new
        self.Vyhat = Vyhat_new
        self.Vhat = Vhat

        self.xhat = xhat_new
        self.yhat = yhat_new
        self.Chat = Chat_new
        self.Shat = Shat_new
        self.psihat = psihat
        

        self.msg.Vxhat = self.Vxhat
        self.msg.Vyhat = self.Vyhat
        self.msg.zhat1 = zhat1_new
        self.msg.zhat2 = zhat2_new
        self.msg.xhat = self.xhat
        self.msg.yhat = self.yhat
        self.msg.Vhat = self.Vhat
        self.msg.psihat = psihat
        pub_Obs.publish(self.msg)

    def Imue_callback(self,sub_Imue):
        self.w = sub_Imue.angular_velocity.z
        self.w = self.w + 0.0171


    def Enc_callback(self,sub_Enc):
        self.xreal = sub_Enc.pose.pose.position.x
        self.yreal = sub_Enc.pose.pose.position.y

    def Imu_callback(self,sub_Imu):
        self.areal = sub_Imu.linear_acceleration.x
        self.areal = self.areal + 0.2924
    
    def Init(self):
        self.Vxhat = 0
        self.Vyhat = 0
        self.Vhat = 0
        self.xhat = 0
        self.yhat = 0
        self.zhat1 = 0
        self.zhat2 = 0
        self.areal = 0
        self.xreal = 0
        self.yreal = 0
        self.psihat = 0
        self.Chat = 1
        self.Shat = 0
        self.u1 = 0
        self.u2 = 0
        self.msg = Obsrv_cmd()
        self.w = 0
if __name__ == '__main__':
    try:
        Obsrv()
    except rospy.ROSInterruptException:
        pass 
        
        
        
    
