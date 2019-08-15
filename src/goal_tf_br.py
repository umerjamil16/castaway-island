#!/usr/bin/env python  
###################################
# TESTING PURPOSES 
#
###################################
import roslib
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        br.sendTransform((0.0, 2.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            rospy.Time.now(),
            "map",
            "pak123")

rate.spin()