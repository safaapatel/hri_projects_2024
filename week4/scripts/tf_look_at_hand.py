#!/usr/bin/env python3
import rospy
import math
import tf2_ros
from sensor_msgs.msg import JointState

# Define global variables for yaw and pitch
current_yaw = 0.0
current_pitch = 0.0
target_yaw = 0.0
target_pitch = 0.0

def get_hand_transform(tfBuffer):
    try:
        trans = tfBuffer.lookup_transform('Head', 'l_gripper', rospy.Time())
        print(f"Hand transform received: {trans.transform.translation}")
        return trans
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return None

def calculate_head_angles(hand_position):
    global current_yaw, current_pitch
    yaw = math.atan2(hand_position.y, hand_position.x)
    pitch = -math.atan2(hand_position.z, math.sqrt(hand_position.x**2 + hand_position.y**2))
    print(f"Calculated Yaw: {yaw}, Pitch: {pitch}")
    return yaw, pitch

def gesture_toward_hand(pub, tfBuffer):
    global current_yaw, current_pitch
    rate = rospy.Rate(2000000)  # 10 Hz rate

    while not rospy.is_shutdown():
        hand_transform = get_hand_transform(tfBuffer)
        if hand_transform:
            hand_position = hand_transform.transform.translation
            current_yaw, current_pitch = calculate_head_angles(hand_position)
            print(f"Publishing Yaw: {current_yaw}, Pitch: {current_pitch}")
        else:
            continue

        # Publish the joint state message
        gesture = JointState()
        gesture.header.stamp = rospy.Time.now()
        gesture.name = ['HeadYaw', 'HeadPitch']
        gesture.position = [current_yaw, current_pitch]
        pub.publish(gesture)

        rate.sleep()

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('tf2_look_at_hand')

        # Create the TF buffer and listener
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        # Create the publisher for joint states
        pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # Run the gesture toward hand function
        gesture_toward_hand(pub, tfBuffer)

    except rospy.ROSInterruptException:
        pass

