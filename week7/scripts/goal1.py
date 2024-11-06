#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    # Get the closest reading by finding the minimum value
    closest_distance = min(msg.ranges)
    
    # Print the closest distance
    rospy.loginfo("Closest obstacle distance: {:.2f} meters".format(closest_distance))

def listener():
    rospy.init_node('closest_obstacle_listener', anonymous=True)
    rospy.Subscriber('/base_scan', LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

