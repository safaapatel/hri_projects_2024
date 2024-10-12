#!/usr/bin/python3
import rospy
from sensor_msgs.msg import JointState
import math

def initialize_joint_state():
    js = JointState()
    js.header.frame_id = "Torso"
    js.name = [
        "HeadYaw", "HeadPitch", "LHipYawPitch", "LHipRoll", "LHipPitch", 
        "LKneePitch", "LAnklePitch", "LAnkleRoll", "LShoulderPitch", 
        "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", 
        "LHand", "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", 
        "RAnklePitch", "RAnkleRoll", "RShoulderPitch", "RShoulderRoll", 
        "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"
    ]
    js.position = [0.0] * len(js.name)
    return js

def publish_joint_states(pub, js):
    js.header.stamp = rospy.get_rostime()
    pub.publish(js)
    rospy.loginfo(js)

def move_to_position(pub, js, position):
    for joint, angle in position.items():
        if joint in js.name:
            js.position[js.name.index(joint)] = math.radians(angle)
    publish_joint_states(pub, js)

# Normal actions
def normal_wave(pub, js):
    positions = [
        {"LShoulderPitch": -45, "LShoulderRoll": 0, "LElbowRoll": 0, "LHand": 80},
        {"LShoulderPitch": -45, "LShoulderRoll": 30, "LElbowRoll": -30, "LHand": 80},
        {"LShoulderPitch": -45, "LShoulderRoll": -30, "LElbowRoll": -30, "LHand": 80},
        {"LShoulderPitch": 0, "LShoulderRoll": 0, "LElbowRoll": 0, "LHand": 0}
    ]
    for position in positions:
        move_to_position(pub, js, position)
        rospy.sleep(1)

def normal_head_shake(pub, js):
    positions = [
        {"HeadYaw": 30},
        {"HeadYaw": -30},
        {"HeadYaw": 0}
    ]
    for position in positions:
        move_to_position(pub, js, position)
        rospy.sleep(0.5)

def normal_head_nod(pub, js):
    positions = [
        {"HeadPitch": 20},
        {"HeadPitch": 0}
    ]
    for position in positions:
        move_to_position(pub, js, position)
        rospy.sleep(0.5)

# Interpolated actions
def perform_action_with_stops(pub, js, start_position, end_position, num_steps, pause_time):
    for step in range(num_steps + 1):
        intermediate_position = {}
        for joint in start_position.keys():
            start_angle = start_position[joint]
            end_angle = end_position[joint]
            intermediate_angle = start_angle + (end_angle - start_angle) * (step / num_steps)
            intermediate_position[joint] = intermediate_angle
        move_to_position(pub, js, intermediate_position)
        rospy.sleep(pause_time)

def interpolated_wave(pub, js):
    start = {"LShoulderPitch": 0, "LShoulderRoll": 0, "LElbowRoll": 0, "LHand": 0}
    positions = [
        {"LShoulderPitch": -45, "LShoulderRoll": 0, "LElbowRoll": 0, "LHand": 80},
        {"LShoulderPitch": -45, "LShoulderRoll": 30, "LElbowRoll": -30, "LHand": 80},
        {"LShoulderPitch": -45, "LShoulderRoll": -30, "LElbowRoll": -30, "LHand": 80},
        {"LShoulderPitch": 0, "LShoulderRoll": 0, "LElbowRoll": 0, "LHand": 0}
    ]
    for end in positions:
        perform_action_with_stops(pub, js, start, end, num_steps=3, pause_time=0.3)
        start = end

def interpolated_head_shake(pub, js):
    positions = [
        {"HeadYaw": 0},
        {"HeadYaw": 30},
        {"HeadYaw": -30},
        {"HeadYaw": 0}
    ]
    for i in range(len(positions) - 1):
        perform_action_with_stops(pub, js, positions[i], positions[i+1], num_steps=2, pause_time=0.2)

def interpolated_head_nod(pub, js):
    positions = [
        {"HeadPitch": 0},
        {"HeadPitch": 20},
        {"HeadPitch": 0}
    ]
    for i in range(len(positions) - 1):
        perform_action_with_stops(pub, js, positions[i], positions[i+1], num_steps=2, pause_time=0.2)

if __name__ == '__main__':
    rospy.init_node('robot_movement', anonymous=True)
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    js = initialize_joint_state()
    rospy.sleep(1)  # Allow time for the publisher to initialize

    try:
        print("Performing normal actions")
        normal_wave(pub, js)
        rospy.sleep(1)
        normal_head_shake(pub, js)
        rospy.sleep(1)
        normal_head_nod(pub, js)
        rospy.sleep(2)

        print("Performing interpolated actions")
        interpolated_wave(pub, js)
        rospy.sleep(1)
        interpolated_head_shake(pub, js)
        rospy.sleep(1)
        interpolated_head_nod(pub, js)

    except rospy.ROSInterruptException:
        pass
