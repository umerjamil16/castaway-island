#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None

robot_0_reached = 0
robot_1_reached = 0

thres1 = 1
thres = 1
shark_speed = 0
shark_speed_w = 0
v_x = 0
w_z = 0

past = 0

def clbk_laser(msg):
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }
    
    take_action(regions)
   
def clbk_laser0(msg):
    global robot_0_reached
    t =  min(msg.ranges[355:365])
    if t < 0.2:
        now = rospy.Time.now()
        time_taken = now - past
        rospy.loginfo("Robot 0 has reached the boundry. Time taken: " + str(time_taken.secs))
        robot_0_reached = 1



def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0
    
    global thres, v_x, w_z, shark_speed, shark_speed_w

    state_description = ''
    
    if regions['front'] > thres1 and regions['fleft'] > thres1 and regions['fright'] > thres:
        state_description = 'case 1 - nothing' + " w: " + str(v_x)
        linear_x = 0.1 * v_x
        shark_speed = linear_x
#        if w_z < 
        angular_z = 0.5*w_z
        shark_speed_w = angular_z
    elif regions['front'] < thres1 and regions['fleft'] > thres1 and regions['fright'] > thres:
        state_description = 'case 2 - front'
        linear_x = 0
        angular_z = 0.1
    elif regions['front'] > thres1 and regions['fleft'] > thres1 and regions['fright'] < thres:
        state_description = 'case 3 - fright'
        linear_x = 0
        angular_z = 0.1
    elif regions['front'] > thres1 and regions['fleft'] < thres1 and regions['fright'] > thres:
        state_description = 'case 4 - fleft'
        linear_x = 0
        angular_z = -0.1
    elif regions['front'] < thres1 and regions['fleft'] > thres1 and regions['fright'] < thres:
        state_description = 'case 5 - front and fright'
        linear_x = 0
        angular_z = 0.1
    elif regions['front'] < thres1 and regions['fleft'] < thres1 and regions['fright'] > thres:
        state_description = 'case 6 - front and fleft'
        linear_x = 0
        angular_z = -0.1
    elif regions['front'] < thres1 and regions['fleft'] < thres1 and regions['fright'] < thres:
        state_description = 'case 7 - front and fleft and fright'
        linear_x = 0
        angular_z = 0.1
    elif regions['front'] > thres1 and regions['fleft'] < thres1 and regions['fright'] < thres:
        state_description = 'case 8 - fleft and fright'
        linear_x = 0.1
        angular_z = 0
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('tf_turtle')
    global pub, v_x, w_z, past
    past = rospy.Time.now()

    listener = tf.TransformListener()
    pub = rospy.Publisher('robot_1/cmd_vel', geometry_msgs.msg.Twist)
    per_vel = rospy.Publisher('robot_0/cmd_vel', geometry_msgs.msg.Twist)
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now() + rospy.Duration(0.0)
            listener.waitForTransform("/robot_1/base_link", "/robot_0/base_link", now, rospy.Duration(10000))
            listener.waitForTransform("/robot_0/base_link", "/robot_1/base_link", now, rospy.Duration(10000))

            (trans,rot) = listener.lookupTransform('/robot_1/base_link', '/robot_0/base_link', rospy.Time(0))

            (trans1,rot1) = listener.lookupTransform('/robot_0/base_link', '/robot_1/base_link', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("TF not found")            
            continue


        w_z = 1 * math.atan2(trans[1], trans[0])
        v_x = 4.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        print "speed: " + str(v_x) + " z: " + str(w_z)
        d1 = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        d2 = math.sqrt(trans1[0] ** 2 + trans1[1] ** 2)

        print "Distance: d1: " + str(d1) + " d2: " + str(d2)

        sub = rospy.Subscriber('/robot_1/base_scan', LaserScan, clbk_laser)
        sub = rospy.Subscriber('/robot_0/base_scan', LaserScan, clbk_laser0)
        
        if robot_0_reached == 0:
            vel_msg1 = geometry_msgs.msg.Twist()
            vel_msg1.angular.z = -1 * math.atan2(trans1[1], trans1[0])
            vel_msg1.linear.x = shark_speed/4#4.5 * math.sqrt(trans1[0] ** 2 + trans1[1] ** 2)
            per_vel.publish(vel_msg1)
    
        
        rate.sleep()
    #rospy.spin()
