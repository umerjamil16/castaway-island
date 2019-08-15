#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf_turtle')

    listener = tf.TransformListener()
    per_vel = rospy.Publisher('robot_0/cmd_vel', geometry_msgs.msg.Twist)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now() + rospy.Duration(0.0)
            listener.waitForTransform("/robot_0/base_link", "/robot_1/base_link", now, rospy.Duration(10000))
            (trans,rot) = listener.lookupTransform('/robot_0/base_link', '/robot_1/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("TF not found")            
            continue


        #print "TRANS: "
        #print trans
        
        vel_msg = geometry_msgs.msg.Twist()
        vel_msg.angular.z = -1 * math.atan2(trans[1], trans[0])
        vel_msg.linear.x = 4.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        per_vel.publish(vel_msg)
        
#        rate.sleep()
    rospy.spin()
