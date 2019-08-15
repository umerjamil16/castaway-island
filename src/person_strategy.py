#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None

thres = 1

v_x = 0
w_z = 0
def clbk_laser(msg):
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }
    
    take_action(regions)
    
def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    global thres, v_x, w_z

    state_description = ''
    
    if regions['front'] > thres and regions['fleft'] > thres and regions['fright'] > thres:
        state_description = 'case 1 - nothing' + " w: " + str(w_z)
        linear_x = 0.1 * v_x
#        if w_z < 
        angular_z = 0.4*w_z
    elif regions['front'] < thres and regions['fleft'] > thres and regions['fright'] > thres:
        state_description = 'case 2 - front'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] > thres and regions['fleft'] > thres and regions['fright'] < thres:
        state_description = 'case 3 - fright'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] > thres and regions['fleft'] < thres and regions['fright'] > thres:
        state_description = 'case 4 - fleft'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] < thres and regions['fleft'] > thres and regions['fright'] < thres:
        state_description = 'case 5 - front and fright'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] < thres and regions['fleft'] < thres and regions['fright'] > thres:
        state_description = 'case 6 - front and fleft'
        linear_x = 0
        angular_z = -0.3
    elif regions['front'] < thres and regions['fleft'] < thres and regions['fright'] < thres:
        state_description = 'case 7 - front and fleft and fright'
        linear_x = 0
        angular_z = 0.3
    elif regions['front'] > thres and regions['fleft'] < thres and regions['fright'] < thres:
        state_description = 'case 8 - fleft and fright'
        linear_x = 0.3
        angular_z = 0
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description )
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('tf_turtle1')
    #global pub, v_x, w_z

    listener = tf.TransformListener()
    pub = rospy.Publisher('robot_1/cmd_vel', geometry_msgs.msg.Twist)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now() + rospy.Duration(0.0)
            listener.waitForTransform("/robot_1/base_link", "/robot_0/base_link", now, rospy.Duration(10000))
            (trans,rot) = listener.lookupTransform('/robot_1/base_link', '/robot_0/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("TF not found")            
            continue


        vel_msg = geometry_msgs.msg.Twist()
        w_z = 1 * math.atan2(trans[1], trans[0])
        v_x = 4.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        print "speed: " + str(v_x) + " z: " + str(w_z)

        sub = rospy.Subscriber('/robot_1/base_scan', LaserScan, clbk_laser)
    
        
        rate.sleep()
    #rospy.spin()
