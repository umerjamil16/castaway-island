#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from ca_main.srv import GoalCustomSrv, GoalCustomSrvRequest
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None

# /robot_0/odom subscriber callback
def sub0_callback(msg):
    global r0_x, r0_y
    pose_q = msg.pose.pose.position
    r0_x = pose_q.x
    r0_y = pose_q.y

# /robot_0/base_scan subscriber callback
def clbk_laser(msg):
    global shark_speed
    laser_reading =  (msg.ranges[540])
    global dist
    if (laser_reading < 0.13 or laser_reading > 5.45):
        if dist > 0.30:
            global past_time
            now = rospy.get_rostime()
            net_time =  now - past_time
            rospy.loginfo("CASTAWAY REACHED BOUNDRY FIRST IN " + str(net_time.secs) + " SECS")
            shark_speed = 0
            rospy.signal_shutdown('Quit')
    else:
        rospy.loginfo("Castaway moving towards boundry")
 
if __name__ == '__main__':
    rospy.init_node('castaway_strategy')
    past_time = 0 # for time stamp
    iter_loop = 0 # to keep track the iteration of a while loop, used to update past_time only once in while loop

    dist = 0 #represents distance between robot_0 and robot_1
    r0_x = 0 #robot_x position
    r0_y = 0 #robot_y position

    island_radius = 3.0 # radius of island
    shark_speed = 0.5 # shark speed

    listener = tf.TransformListener()
    pub0 = rospy.Publisher('robot_0/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
    pub1 = rospy.Publisher('robot_1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now() + rospy.Duration(0.0)
            listener.waitForTransform("/map", "/robot_0/base_link", now, rospy.Duration(10000))
            listener.waitForTransform('/robot_1/base_link', '/robot_0/base_link', now, rospy.Duration(10000))
            (trans,rot) = listener.lookupTransform('/map', '/robot_0/base_link', rospy.Time(0))
            (trans1,rot1) = listener.lookupTransform('/robot_1/base_link', '/robot_0/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("TF not found")            
            continue
        
        if iter_loop == 0: # to update past_time var only once in loop
            past_time = rospy.get_rostime()

        iter_loop = iter_loop + 1

        vel_msg1 = geometry_msgs.msg.Twist()
        vel_msg1.angular.z = 0
        vel_msg1.linear.x = shark_speed/4 # given that castaway speed is 1/4 of shark speed
        pub0.publish(vel_msg1)

        d2 = math.sqrt(trans1[0] ** 2 + trans1[1] ** 2)
        dist = d2
        d1 = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        dist_bw_shark_ca = island_radius - d1 + 0.03 # 0.03 is the upper tolerance

        vel_msg = geometry_msgs.msg.Twist()
        if d2 > dist_bw_shark_ca:
            rospy.loginfo("Shark moving towards castaway")
            rate = rospy.Rate(10.0)            
            if r0_x > 0:
                vel_msg.linear.x = shark_speed
                vel_msg.angular.z = -1 * shark_speed/island_radius # v = r * omega implementation
            elif r0_x < 0:
                vel_msg.linear.x = -1 * shark_speed
                vel_msg.angular.z = shark_speed/island_radius
            else:
                vel_msg.linear.x = shark_speed
                vel_msg.angular.z = -1 * shark_speed/island_radius
        else:
            now = rospy.get_rostime()
            net_time1 =  now - past_time
            rospy.loginfo("SHARK REACHED IN FRONT OF CASTAWAY IN " + str(net_time1.secs)  + " SECS")
            rospy.loginfo("CASTAWAY CANNOT ESCAPE, MOVE THE CASTAWAY TO ANOTHER LOCATION AND TRY AGAIN")
            rospy.signal_shutdown('Quit')

        rate = rospy.Rate(10.0)


        pub1.publish(vel_msg)
        sub_laser_scanner = rospy.Subscriber('/robot_0/base_scan', LaserScan, clbk_laser)
        sub0 = rospy.Subscriber ('/robot_0/odom', Odometry, sub0_callback)


        rate.sleep()