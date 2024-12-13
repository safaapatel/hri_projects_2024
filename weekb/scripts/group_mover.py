#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

group_type = None

def group_callback(msg):
    global group_type
    group_type = msg.data
    rospy.loginfo(f"Detected group type: {group_type}")

def move_robot():
    global group_type
    pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        twist = Twist()
        
        if group_type == "circle":
            rospy.loginfo("Moving towards the center of the circle.")
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            
        elif group_type == "line":
            rospy.loginfo("Moving to the end of the line.")
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            
        else:
            rospy.loginfo("No group detected. Stopping the robot.")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('group_movement_node')
        rospy.Subscriber("/robot_0/detected_groups", String, group_callback)
        move_robot()
    except rospy.ROSInterruptException:
        pass

