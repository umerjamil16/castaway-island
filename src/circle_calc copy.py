#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None


if __name__ == '__main__':
    rospy.init_node('tf_turtle1123')
    #global pub, v_x, w_z

    listener = tf.TransformListener()
    pub = rospy.Publisher('robot_1/cmd_vel', geometry_msgs.msg.Twist)

    rate = rospy.Rate(10.0)
    vel_msg = geometry_msgs.msg.Twist()
    vel_msg.linear.x = -0.5
    vel_msg.angular.z = +0.1612903
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        pub.publish(vel_msg)
   
        rate.sleep()
#    rospy.spin()
