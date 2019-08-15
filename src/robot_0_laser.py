#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None

thres = 1

def clbk_laser(msg):
    print " LASER READING FRONT"
    print (msg.ranges[540])

def main():
    global pub
    
    rospy.init_node('eobot_0_laser_reading_test')
    
   
    sub = rospy.Subscriber('/robot_0/base_scan', LaserScan, clbk_laser)
    
    rospy.spin()

if __name__ == '__main__':
    main()
