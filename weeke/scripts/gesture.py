#!/usr/bin/python3
import rospy
import math
from sensor_msgs.msg import JointState
import subprocess
from std_msgs.msg import String
import re

received_command = ""

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

def phrase_callback(data):
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rate = rospy.Rate(20)
    global received_command
    match_greeting = "hello"
    match_salutation = "hi"
    match_positive = "yes"
    match_negative = "no"
    
    received_command = data.data
    rospy.loginfo(f"Received Command: {received_command}")
    subprocess.run(['espeak', received_command])
    
    try:
        if re.search(rf'\b{match_greeting}\b', received_command.lower()):
            normal_wave(pub, js)
        elif re.search(rf'\b{match_salutation}\b', received_command.lower()):
            normal_wave(pub, js)
        elif re.search(rf'\b{match_positive}\b', received_command.lower()):
            normal_head_nod(pub, js)
        elif re.search(rf'\b{match_negative}\b', received_command.lower()):
            normal_head_shake(pub, js)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    rospy.Subscriber('/tts_phrase', String, phrase_callback)
    rospy.init_node('talker', anonymous=True)
    js = initialize_joint_state()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

