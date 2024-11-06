#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/base_scan', LaserScan, self.scan_callback)
        
        self.min_distance = 0.5    
        self.forward_speed = 0.2   
        self.turning_speed = 0.5   
        
        self.is_turning = False
        self.turn_direction = 1  
    
    def get_average_distance(self, ranges, start_idx, end_idx):
        valid_ranges = [r for r in ranges[start_idx:end_idx] 
                       if r > 0 and not math.isinf(r) and not math.isnan(r)]
        if valid_ranges:
            return sum(valid_ranges) / len(valid_ranges)
        return 0
    
    def scan_callback(self, msg):
        twist = Twist()
        
        total_ranges = len(msg.ranges)
        left_ranges = msg.ranges[:total_ranges//3]
        front_ranges = msg.ranges[total_ranges//3:2*total_ranges//3]
        right_ranges = msg.ranges[2*total_ranges//3:]
        
        valid_front = [r for r in front_ranges if r > 0 and not math.isinf(r) and not math.isnan(r)]
        min_front = min(valid_front) if valid_front else 0
        
        left_avg = self.get_average_distance(msg.ranges, 0, total_ranges//3)
        right_avg = self.get_average_distance(msg.ranges, 2*total_ranges//3, total_ranges)
        
        rospy.loginfo(f"Front: {min_front:.2f}, Left: {left_avg:.2f}, Right: {right_avg:.2f}")
        
        if min_front > self.min_distance and not self.is_turning:
            twist.linear.x = self.forward_speed
            twist.angular.z = 0
            rospy.loginfo("Moving forward")
        else:
            self.is_turning = True
            
            if not self.is_turning:
                self.turn_direction = 1 if left_avg > right_avg else -1
            
            twist.linear.x = 0
            twist.angular.z = self.turning_speed * self.turn_direction
            
            if min_front > self.min_distance:
                forward_clear = True
                if self.turn_direction > 0:
                    left_front = self.get_average_distance(msg.ranges, total_ranges//6, total_ranges//3)
                    forward_clear = left_front > self.min_distance
                else:
                    right_front = self.get_average_distance(msg.ranges, 2*total_ranges//3, 5*total_ranges//6)
                    forward_clear = right_front > self.min_distance
                
                if forward_clear:
                    self.is_turning = False
                    rospy.loginfo("Turn complete")
            
            rospy.loginfo(f"Turning {'left' if self.turn_direction > 0 else 'right'}")
        
        if min(valid_front) < self.min_distance/2 if valid_front else False:
            twist.linear.x = 0
            rospy.loginfo("Emergency stop - too close!")
        
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        node = ObstacleAvoidance()
        rospy.loginfo("Obstacle avoidance node started")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

