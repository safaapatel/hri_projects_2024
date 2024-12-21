#!/usr/bin/env python

import rospy
import subprocess
import pyaudio
from std_msgs.msg import String
from ros_vosk.msg import speech_recognition
from vosk import Model, KaldiRecognizer


class SpeechRecognitionNode:
    def __init__(self):
        rospy.init_node('speech_recognition_node', anonymous=True)
        self.pub = rospy.Publisher('/tts_phrase', String, queue_size=10)
        self.model = self.load_vosk_model()
        self.recognizer = KaldiRecognizer(self.model, 16000)
        self.stream = self.setup_microphone_stream()

    def load_vosk_model(self):
        try:
            return Model(model_name="vosk-model-small-en-us-0.15")
        except Exception as e:
            rospy.logerr(f"Error loading model: {e}")
            raise

    def setup_microphone_stream(self):
        p = pyaudio.PyAudio()
        try:
            stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000,
                            input=True, frames_per_buffer=8000)
            stream.start_stream()
            rospy.loginfo("Microphone stream started. Listening...")
            return stream
        except IOError as e:
            rospy.logerr(f"Error initializing microphone stream: {e}")
            raise

    def process_audio(self):
        while not rospy.is_shutdown():
            audio_data = self.stream.read(4000, exception_on_overflow=False)

            if len(audio_data) == 0:
                continue

            if self.recognizer.AcceptWaveform(audio_data):
                result = self.recognizer.Result()
                recognized_text = result.split('"')[3]
                rospy.loginfo(f"Recognized speech: {recognized_text}")
                self.pub.publish(recognized_text)
                self.speak_text(recognized_text)

    def speak_text(self, text):
        try:
            subprocess.run(['espeak', text])
        except Exception as e:
            rospy.logerr(f"Error with TTS: {e}")


def main():
    try:
        speech_recognition_node = SpeechRecognitionNode()
        speech_recognition_node.process_audio()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception caught. Shutting down.")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")


if __name__ == '__main__':
    main()

