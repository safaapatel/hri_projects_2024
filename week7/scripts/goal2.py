#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math

class ShapeMovementNode:
    def __init__(self):
        rospy.init_node('shape_movement_node')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz

    def move(self, linear=0, angular=0, duration=1):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < duration:
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()
        
        self.stop()
        rospy.sleep(0.5)
        
    def stop(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
    def move_square(self, side_length=1.0):
        rospy.loginfo("Starting square movement")
        linear_speed = 0.2
        angular_speed = 0.5
        
        for i in range(4):
            rospy.loginfo(f"Side {i+1}")
            self.move(linear=linear_speed, angular=0, duration=side_length/linear_speed)
            self.move(linear=0, angular=angular_speed, duration=(math.pi/2)/angular_speed)
        
        rospy.loginfo("Square completed")
            
    def move_figure_eight(self, radius=1.0):
        rospy.loginfo("Starting figure eight movement")
        linear_speed = 0.2
        angular_speed = 0.2
        circle_duration = (2 * math.pi) / angular_speed

        self.move(linear=linear_speed, angular=-angular_speed, duration=circle_duration)
        self.move(linear=linear_speed, angular=angular_speed, duration=circle_duration)
        
        rospy.loginfo("Figure eight completed")
            
    def move_triangle(self, side_length=1.0):
        rospy.loginfo("Starting triangle movement")
        linear_speed = 0.2
        angular_speed = 0.5
        
        for i in range(3):
            rospy.loginfo(f"Side {i+1}")
            self.move(linear=linear_speed, angular=0, duration=side_length/linear_speed)
            self.move(linear=0, angular=angular_speed, duration=(2*math.pi/3)/angular_speed)
        
        rospy.loginfo("Triangle completed")

if __name__ == '__main__':
    try:
        node = ShapeMovementNode()
        
        while not rospy.is_shutdown():
            print("\nSelect a shape to draw:")
            print("1. Square")
            print("2. Figure Eight")
            print("3. Triangle")
            print("4. Exit")
            
            choice = input("Enter your choice (1-4): ")
            
            if choice == '1':
                node.move_square(side_length=1.0)
            elif choice == '2':
                node.move_figure_eight(radius=1.0)
            elif choice == '3':
                node.move_triangle(side_length=1.0)
            elif choice == '4':
                break
            else:
                print("Invalid choice. Please try again.")
                
    except rospy.ROSInterruptException:
        pass

