#!/usr/bin/env python3
import rospy
import math
import tf2_ros
import threading
from sensor_msgs.msg import JointState

def smooth_transition(current, target, alpha=0.1):
    return current + alpha * (target - current)

class HeadTracker:
    def __init__(self):
        rospy.init_node('nao_head_tracker')

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        self.current_yaw = 0.0
        self.current_pitch = 0.0
        self.target_yaw = 0.0
        self.target_pitch = 0.0

        #self.lock = threading.Lock()  
        
    def get_hand_transform(self):
        try:
            trans = self.tfBuffer.lookup_transform('torso', 'l_gripper', rospy.Time())
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def calculate_head_angles(self, hand_position):
        yaw = math.atan2(hand_position.y, hand_position.x)
        pitch = math.atan2(-hand_position.z, math.sqrt(hand_position.x**2 + hand_position.y**2))
        return yaw, pitch

    def gesture_toward_hand(self):
        rate = rospy.Rate(10000.0)
        
        while not rospy.is_shutdown():
            hand_transform = self.get_hand_transform()
            if hand_transform:
                hand_position = hand_transform.transform.translation
                self.current_yaw, self.current_pitch = self.calculate_head_angles(hand_position)
            else:
            	continue
            

            

            gesture = JointState()
            gesture.header.stamp = rospy.Time.now()
            gesture.name = ['HeadYaw', 'HeadPitch']
            gesture.position = [self.current_yaw, self.current_pitch]
            self.pub.publish(gesture)

            #rospy.sleep(1)  # Hold the gesture for a moment

            
            rate.sleep()

    
    

    def run(self):
        threading.Thread(target=self.gesture_toward_hand).start()
        #threading.Thread(target=self.look_in_hand_direction).start()

if __name__ == '__main__':
    try:
        tracker = HeadTracker()
        tracker.run()
        #rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass

