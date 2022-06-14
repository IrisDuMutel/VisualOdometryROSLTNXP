#!/usr/bin/env python3

# This is the code from the APF Enza did on MATLAB but translated to Pyhton by Iris
# This APF can only deal with one obstacle. Otherwise, create a message with lists of 
# obstacle coordinates


from numpy import linalg as LA
import math
import queue
import rospy
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from visual_odometry.msg import PWM_cmd,APF_cmd


# Potential field characteristics
rho_0 = 0.3;                   # Area of influence of obstacle  [m] #0.5
kp    = 1;                  # Attractive potential gain #0.15
eta   = 0.5;                  # Repulsive potential gain  #0.05

class APF():
    
    def __init__(self):
        rospy.init_node('APF',anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
        sub_destination = message_filters.Subscriber("/goal", Pose, queue_size = 10)
        sub_obstacle = message_filters.Subscriber("/obstacle", Pose, queue_size = 10)
        sub_odometry    = message_filters.Subscriber("/odometry/filtered", Odometry, queue_size = 10)
        self.pub = rospy.Publisher('APF_output',APF_cmd,queue_size=10)

        self.msg = APF_cmd()
        ts = message_filters.ApproximateTimeSynchronizer([sub_destination,sub_obstacle,sub_odometry], queue_size=10, slop=0.5, allow_headerless=True)
        ts.registerCallback(self.callback)


        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        
    def callback(self,sub_destination, sub_obstacle, sub_odometry):

        X_est = sub_odometry.pose.pose.position.x
        Y_est = sub_odometry.pose.pose.position.y
        X_goal = sub_destination.position.x
        Y_goal = sub_destination.position.y
        X_obstacle = sub_obstacle.position.x
        Y_obstacle = sub_obstacle.position.y
        q=[X_est, Y_est]           # Actual position [m]       
        qg=[X_goal, Y_goal]        # Goal position   [m]
        rho_goal=math.dist(q, qg)  # Distance actual position-goal position
        d=2                        # Distance from goal, point of attractive field characteristic switching
        # ATTRACTIVE
        if rho_goal<=d:
            F_attx = -kp*(X_est-X_goal)
            F_atty = -kp*(Y_est-Y_goal)
        else:
            F_attx = -kp*(X_est-X_goal)*d/rho_goal
            F_atty = -kp*(Y_est-Y_goal)*d/rho_goal
        
        # REPULSIVE
        F_repx = 0
        F_repy = 0
        dmin   = 10000
        qc     = [0,0]
        obst   = 0

        
        qc=[X_obstacle, Y_obstacle]              # Obstacle position [m]
        dq = math.dist(q,qc)                           # Distance actual position- obstacle position
        
        if dmin >= dq:
            dmin    = dq
            qc      = [X_obstacle, Y_obstacle]
            rho     = math.dist(q,qc) # Distance with the nearest obstacle
            
            if rho<=rho_0:                          # Inside obstacle area! Danger
                F_repx = eta*(1/rho - 1/rho_0)*1/(rho)**2*(X_est-X_obstacle)/math.dist(q,qc)
                F_repy = eta*(1/rho - 1/rho_0)*1/(rho)**2*(Y_est-Y_obstacle)/math.dist(q,qc)
                obst=1/rho
        # To find the goal if the obstacle is near
        dob=0.3
        dgr=0.3

        if math.dist(q,qc)<=dob and math.dist(q,qg)<=dgr:
            F_totx = F_attx
            F_toty = F_atty
        else:
            F_totx = F_attx+F_repx
            F_toty = F_atty+F_repy
        

        # Potential Force
        F_tot = LA.norm([F_totx,F_toty])

        # ORIENTATION REFERENCE SIGNAL
        psi_ref=math.atan2(F_toty,F_totx) 

        # 2° prova per il campo di velocità
        delta=0.6/rho_goal+obst   #0.4
        V_ref= 0.5*2/(2+delta)

        # END OF SIMULATION
        if abs(rho_goal)<=0.1:
            V_ref=0
            # psi_ref=0
            eos=1
        else:
            eos=0
        
        # ROS message and publisher
        self.msg.Vref = V_ref
        self.msg.Psiref = psi_ref
        self.pub.publish(self.msg)
        
        # Mathematical minimum
        # if F_tot==0:
        #     psi_ref=0
        #     V_ref=-0.3
        
        # self.rate.sleep()

if __name__ == '__main__':
    try:
        APF()
    except rospy.ROSInterruptException:
        pass
