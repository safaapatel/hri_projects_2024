#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('simple_obstacle_avoidance')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/base_scan', LaserScan, self.scan_callback)
        self.min_distance = 0.5  # Distance to obstacle before stopping
        self.obstacle_detected = False  # Flag to ensure obstacle message is printed only once
        
    def scan_callback(self, msg):
        front_readings = msg.ranges[len(msg.ranges)//4:3*len(msg.ranges)//4]
        closest_front = min([r for r in front_readings if r > 0 and not math.isinf(r)])
        
        twist = Twist()
        
        if closest_front > self.min_distance:
            if not self.obstacle_detected:  # Only print once when moving forward
                rospy.loginfo("Moving forward")
                self.obstacle_detected = False  # Reset the obstacle flag when moving forward
            twist.linear.x = 0.2  # Move forward
        else:
            if not self.obstacle_detected:  # Print obstacle message only once
                rospy.loginfo("Obstacle detected! Stopping robot.")
                self.obstacle_detected = True  # Set the flag to True after printing
            twist.linear.x = 0  # Stop the robot

        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        node = ObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

