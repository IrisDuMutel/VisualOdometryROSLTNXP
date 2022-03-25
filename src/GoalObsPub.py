#!/usr/bin/env python3
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose

def GoalObsPub():
    pub_pose = rospy.Publisher('goal', Pose, queue_size=10)
    pub_obstacle = rospy.Publisher('obstacle', Pose, queue_size=10)
    rospy.init_node('GoalObsPublisher', anonymous=True)
    goal = Pose()
    obstacle = Pose()
    goal.position.x = 2.5
    goal.position.y = 0
    obstacle.position.x = 1
    obstacle.position.y = 0
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub_pose.publish(goal)
        pub_obstacle.publish(obstacle)
        rate.sleep()

if __name__ == '__main__':
    try:
        GoalObsPub()
    except rospy.ROSInterruptException:
        pass
