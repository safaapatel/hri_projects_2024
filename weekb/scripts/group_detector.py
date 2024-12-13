#!/usr/bin/env python3
import math
import rospy
from people_msgs.msg import PositionMeasurementArray
from geometry_msgs.msg import Point
from std_msgs.msg import String

def calculate_distance(p1, p2):
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

def detect_group(people_data):
    positions = [person.pos for person in people_data.people]
    x_positions = [pos.x for pos in positions]
    y_positions = [pos.y for pos in positions]
    x_variation = max(x_positions) - min(x_positions)
    y_variation = max(y_positions) - min(y_positions)
    
    if x_variation < 3 and y_variation > 4:
        return "line"
    
    center = Point(x=0, y=0, z=0)
    distances = [calculate_distance(center, pos) for pos in positions]
    
    if max(distances) - min(distances) < 2:
        return "circle"
    
    return "unknown"

def people_callback(msg):
    rospy.loginfo(f"Received {len(msg.people)} people data")
    group_type = detect_group(msg)
    rospy.loginfo(f"Detected group type: {group_type}")
    detected_groups_pub.publish(group_type)

if __name__ == '__main__':
    try:
        rospy.init_node('group_detector_node')
        detected_groups_pub = rospy.Publisher('/robot_0/detected_groups', String, queue_size=10)
        rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, people_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


