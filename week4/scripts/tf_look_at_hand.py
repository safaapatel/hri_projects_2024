#!/usr/bin/env python3
import rospy
import math
import tf2_ros
from sensor_msgs.msg import JointState

# Define global variables for joint states
current_yaw = 0.0
current_pitch = 0.0
other_joint_states = {}
is_tracking = True  # Flag to indicate if we're actively tracking

def joint_states_callback(msg):
    """Callback function to receive joint states from joint_state_publisher_gui."""
    global other_joint_states
    # Process non-head joints to prevent interference with tracking
    for i, name in enumerate(msg.name):
        if name not in ['HeadYaw', 'HeadPitch']:
            other_joint_states[name] = msg.position[i]
    rospy.loginfo("Received joints from GUI: %s", other_joint_states.keys())

def get_hand_transform(tfBuffer):
    """Get the current transform from torso to l_gripper."""
    try:
        trans = tfBuffer.lookup_transform('torso', 'l_gripper', rospy.Time())
        rospy.loginfo(f"Hand transform received: {trans.transform.translation}")
        return trans
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Could not find transform from torso to l_gripper")
        return None

def calculate_head_angles(hand_position):
    """Calculate head yaw and pitch angles based on hand position."""
    yaw = math.atan2(hand_position.y, hand_position.x)
    pitch = -math.atan2(hand_position.z, math.sqrt(hand_position.x**2 + hand_position.y**2))
    
    # Add limits to prevent extreme movements
    yaw = max(min(yaw, math.pi / 2), -math.pi / 2)  # Limit yaw to ±90 degrees
    pitch = max(min(pitch, math.pi / 4), -math.pi / 4)  # Limit pitch to ±45 degrees
    
    rospy.loginfo(f"Calculated Yaw: {yaw}, Pitch: {pitch}")
    return yaw, pitch

def gesture_toward_hand(pub, tfBuffer):
    """Continuously track the hand position and update head angles."""
    global current_yaw, current_pitch, other_joint_states
    
    rate = rospy.Rate(10)  # 10 Hz rate
    
    while not rospy.is_shutdown():
        hand_transform = get_hand_transform(tfBuffer)
        if hand_transform:
            hand_position = hand_transform.transform.translation
            current_yaw, current_pitch = calculate_head_angles(hand_position)
            rospy.loginfo(f"Tracking - Yaw: {current_yaw}, Pitch: {current_pitch}")

            # Create and publish joint state message
            gesture = JointState()
            gesture.header.stamp = rospy.Time.now()
            gesture.name = ['HeadYaw', 'HeadPitch'] + list(other_joint_states.keys())
            gesture.position = [current_yaw, current_pitch] + list(other_joint_states.values())

            pub.publish(gesture)
        else:
            rospy.logwarn("No hand transform available, head tracking paused.")

        rate.sleep()

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('tf2_look_at_hand')
        rospy.loginfo("Starting tf2_look_at_hand node")
        
        # Create the TF buffer and listener
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        
        # Create the publisher for joint states
        pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # Subscribe to joint states from joint_state_publisher_gui
        rospy.Subscriber('/joint_states', JointState, joint_states_callback)
        
        # Wait a moment for the subscriber to connect
        rospy.sleep(1)
        rospy.loginfo("Subscribers and publishers initialized")
        
        # Run the gesture toward hand function
        gesture_toward_hand(pub, tfBuffer)
        
    except rospy.ROSInterruptException:
        pass
