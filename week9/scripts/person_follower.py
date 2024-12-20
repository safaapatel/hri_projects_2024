#!/usr/bin/env python3

import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from people_msgs.msg import PositionMeasurementArray
import math

class PersonFollower:
    def __init__(self):
        rospy.init_node('person_follower')
        
        # TF2 setup
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/base_scan', LaserScan, self.scan_callback)
        self.person_sub = rospy.Subscriber('/people_tracker_measurements', 
                                         PositionMeasurementArray, 
                                         self.person_callback)
        
        # Parameters
        self.min_obstacle_distance = 0.8  # Increased safety margin
        self.max_linear_speed = 0.3
        self.min_linear_speed = 0.1
        self.max_angular_speed = 0.5
        self.target_distance = 1.2  # Increased following distance
        self.close_distance = 0.9   # Distance to start slowing down
        self.safety_buffer = 0.4    # Additional buffer around obstacles
        
        # State variables
        self.latest_scan = None
        self.target_position = None
        self.is_turning = False
        self.turn_direction = 1
        self.last_target_update = rospy.Time.now()
        self.target_timeout = rospy.Duration(1.0)  # Timeout for target tracking

    def broadcast_person_tf(self, measurement):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "detected_person"
        
        t.transform.translation.x = measurement.pos.x
        t.transform.translation.y = measurement.pos.y
        t.transform.translation.z = 0.0
        
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

    def get_average_distance(self, ranges, start_idx, end_idx):
        valid_ranges = [r for r in ranges[start_idx:end_idx] 
                       if r > 0 and not math.isinf(r) and not math.isnan(r)]
        return sum(valid_ranges) / len(valid_ranges) if valid_ranges else float('inf')

    def person_callback(self, msg):
        if msg.people and len(msg.people) > 0:
            measurement = msg.people[0]
            self.target_position = Point(measurement.pos.x, measurement.pos.y, 0)
            self.broadcast_person_tf(measurement)
            self.last_target_update = rospy.Time.now()
            rospy.loginfo(f"Person detected at: x={measurement.pos.x:.2f}, y={measurement.pos.y:.2f}")

    def get_angle_to_target(self):
        if self.target_position is None:
            return 0
        return math.atan2(self.target_position.y, self.target_position.x)

    def get_distance_to_target(self):
        if self.target_position is None:
            return float('inf')
        return math.sqrt(self.target_position.x**2 + self.target_position.y**2)

    def calculate_speed(self, distance_to_target, min_front_distance):
        """Calculate appropriate linear and angular speeds based on conditions"""
        # Base linear speed calculation
        if distance_to_target < self.close_distance:
            linear_speed = 0
        else:
            # Scale speed based on distance
            speed_factor = min(1.0, (distance_to_target - self.close_distance) / 
                             (self.target_distance - self.close_distance))
            linear_speed = self.min_linear_speed + \
                          (self.max_linear_speed - self.min_linear_speed) * speed_factor

        # Reduce speed when obstacles are nearby
        obstacle_factor = min(1.0, (min_front_distance - self.safety_buffer) / 
                            (self.min_obstacle_distance - self.safety_buffer))
        linear_speed *= max(0.0, obstacle_factor)

        return linear_speed

    def scan_callback(self, msg):
        # Check if target is still valid
        if (rospy.Time.now() - self.last_target_update) > self.target_timeout:
            self.target_position = None
            self.cmd_vel_pub.publish(Twist())
            return

        if self.target_position is None:
            self.cmd_vel_pub.publish(Twist())
            return
            
        twist = Twist()
        
        # Process laser scan data
        total_ranges = len(msg.ranges)
        # Expand front scan area for better obstacle detection
        front_start = int(total_ranges * 0.25)  # 45 degrees
        front_end = int(total_ranges * 0.75)    # 135 degrees
        front_ranges = msg.ranges[front_start:front_end]
        
        valid_front = [r for r in front_ranges if r > 0 and not math.isinf(r) and not math.isnan(r)]
        min_front = min(valid_front) if valid_front else float('inf')
        
        angle_to_target = self.get_angle_to_target()
        distance_to_target = self.get_distance_to_target()
        
        # Enhanced obstacle check
        is_path_clear = min_front > (self.min_obstacle_distance + self.safety_buffer)
        
        if is_path_clear and not self.is_turning:
            # Calculate appropriate speed
            linear_speed = self.calculate_speed(distance_to_target, min_front)
            
            # Adjust angular speed based on angle
            angular_speed = angle_to_target
            if abs(angular_speed) > self.max_angular_speed:
                angular_speed = self.max_angular_speed * (1 if angular_speed > 0 else -1)
            
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
            
            rospy.loginfo(f"Following - Distance: {distance_to_target:.2f}m, Speed: {linear_speed:.2f}m/s")
        else:
            # Obstacle avoidance mode
            self.is_turning = True
            
            left_avg = self.get_average_distance(msg.ranges, 0, total_ranges//3)
            right_avg = self.get_average_distance(msg.ranges, 2*total_ranges//3, total_ranges)
            
            # Choose turn direction based on both obstacle positions and target location
            target_angle = self.get_angle_to_target()
            if not self.is_turning:
                # Consider both clearance and target direction
                if abs(target_angle) > math.pi/2:  # Target is behind
                    self.turn_direction = 1 if left_avg > right_avg else -1
                else:
                    self.turn_direction = 1 if target_angle > 0 else -1
            
            twist.linear.x = 0
            twist.angular.z = self.max_angular_speed * self.turn_direction
            
            if is_path_clear:
                self.is_turning = False
                rospy.loginfo("Path clear - resuming follow")
        
        # Emergency stop if too close
        if min_front < self.safety_buffer:
            twist.linear.x = 0
            twist.angular.z = self.max_angular_speed * self.turn_direction
            rospy.loginfo("Emergency maneuver!")
        
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        node = PersonFollower()
        rospy.loginfo("Person follower node started")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
