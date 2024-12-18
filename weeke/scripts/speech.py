#!/usr/bin/python3
import rospy
from ros_vosk.msg import speech_recognition
from std_msgs.msg import String
import subprocess

class SpeechListener:
    def __init__(self):
        rospy.init_node('speech_listener', anonymous=True)
        self.tts_pub = rospy.Publisher('/tts/phrase', String, queue_size=10)
        self.gesture_pub = rospy.Publisher('/nao/gesture', String, queue_size=10)
        rospy.Subscriber('/speech_recognition', speech_recognition, self.speech_callback)
        rospy.loginfo("Speech Listener Node Initialized")

    def speech_callback(self, msg):
        if msg.type == 'final':
            recognized_text = msg.text.strip()
            if not recognized_text:
                return
            rospy.loginfo(f"Recognized Speech: {recognized_text}")
            self.tts_pub.publish(recognized_text)
            lowercase_text = recognized_text.lower()
            if 'hello' in lowercase_text or 'hi' in lowercase_text:
                self.gesture_pub.publish('wave')
            elif 'yes' in lowercase_text:
                self.gesture_pub.publish('nod')
            elif 'no' in lowercase_text:
                self.gesture_pub.publish('shake')

    def run(self):
        rospy.spin()

def main():
    try:
        listener = SpeechListener()
        listener.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

