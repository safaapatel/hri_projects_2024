#!/usr/bin/python3
import rospy
from sensor_msgs.msg import JointState
import math
import time

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

def move_joint(js, joint_name, angle):
    if joint_name in js.name:
        js.position[js.name.index(joint_name)] = math.radians(angle)

def perform_action(pub, js, action):
    for joint, angle in action.items():
        move_joint(js, joint, angle)
    publish_joint_states(pub, js)
    time.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node('robot_movement', anonymous=True)
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    js = initialize_joint_state()

    rospy.sleep(1)  # Allow time for publisher to set up

    try:
        # Wave
        perform_action(pub, js, {"LShoulderPitch": -45, "LShoulderRoll": 0, "LHand": 80})
        perform_action(pub, js, {"LShoulderPitch": -45, "LShoulderRoll": 45, "LHand": 80})
        perform_action(pub, js, {"LShoulderPitch": -45, "LShoulderRoll": 0, "LHand": 0})

        # Head shake
        perform_action(pub, js, {"HeadYaw": 45})
        perform_action(pub, js, {"HeadYaw": -45})
        perform_action(pub, js, {"HeadYaw": 0})

        # Head nod
        perform_action(pub, js, {"HeadPitch": 30})
        perform_action(pub, js, {"HeadPitch": 0})

    except rospy.ROSInterruptException:
        pass

