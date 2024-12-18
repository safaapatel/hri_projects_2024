#!/usr/bin/python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math

class NaoGestureController:
    def __init__(self):
        rospy.init_node('nao_gesture_controller', anonymous=True)
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        rospy.Subscriber('/nao/gesture', String, self.gesture_callback)
        self.joint_state = JointState()
        self.joint_state.header.frame_id = "Torso"
        self.joint_names = [
            "HeadYaw", "HeadPitch", 
            "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",
            "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll"
        ]
        self.joint_state.name = self.joint_names
        self.joint_state.position = [0.0] * len(self.joint_names)
        rospy.loginfo("Gesture Controller Node Initialized")

    def set_joint_position(self, joint_name, angle_degrees):
        if joint_name in self.joint_state.name:
            index = self.joint_state.name.index(joint_name)
            self.joint_state.position[index] = math.radians(angle_degrees)

    def reset_joints(self):
        for joint in self.joint_names:
            self.set_joint_position(joint, 0)
        self.publish_joints()

    def publish_joints(self):
        self.joint_state.header.stamp = rospy.get_rostime()
        self.joint_pub.publish(self.joint_state)

    def wave_gesture(self):
        wave_sequence = [
            {"RShoulderPitch": -45, "RShoulderRoll": 0, "RElbowRoll": 30},
            {"RShoulderRoll": 20, "RElbowRoll": 50},
            {"RShoulderRoll": -20, "RElbowRoll": 50},
            {"RShoulderRoll": 0, "RElbowRoll": 30}
        ]
        
        for pose in wave_sequence:
            for joint, angle in pose.items():
                self.set_joint_position(joint, angle)
            self.publish_joints()
            rospy.sleep(0.5)
        
        self.reset_joints()

    def nod_gesture(self):
        nod_sequence = [
            {"HeadPitch": 20},
            {"HeadPitch": 0},
            {"HeadPitch": 20},
            {"HeadPitch": 0}
        ]
        
        for pose in nod_sequence:
            for joint, angle in pose.items():
                self.set_joint_position(joint, angle)
            self.publish_joints()
            rospy.sleep(0.5)
        
        self.reset_joints()

    def shake_gesture(self):
        shake_sequence = [
            {"HeadYaw": 30},
            {"HeadYaw": -30},
            {"HeadYaw": 30},
            {"HeadYaw": 0}
        ]
        
        for pose in shake_sequence:
            for joint, angle in pose.items():
                self.set_joint_position(joint, angle)
            self.publish_joints()
            rospy.sleep(0.5)
        
        self.reset_joints()

    def gesture_callback(self, msg):
        gesture = msg.data.lower()
        if gesture == 'wave':
            self.wave_gesture()
        elif gesture == 'nod':
            self.nod_gesture()
        elif gesture == 'shake':
            self.shake_gesture()

    def run(self):
        rospy.spin()

def main():
    try:
        gesture_controller = NaoGestureController()
        gesture_controller.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

