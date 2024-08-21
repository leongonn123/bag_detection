#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
import serial
import time
import math
from vosk import Model, KaldiRecognizer
import pyaudio
import pyttsx3

# Initialize serial port for servos
port = "/dev/ttyUSB0"  # Define particular port used
ser = serial.Serial(port, 1000000)  # Define serial port
ser.close()  # Close port if previously open
ser.open()  # Open port
print(ser.isOpen())  # Make sure we could open the port!

# Define functions to calculate the Goal Position
def GoalPositionValH(angle):
    if math.floor(angle * 3.41) > 256:
        return int(math.floor(angle * 3.41 / 256))
    else:
        return 0

def GoalPositionValL(angle):
    if math.floor(angle * 3.41) < 256:
        return int(math.floor(angle * 3.41))
    else:
        more = int(math.floor(angle * 3.41 / 256))
        return int(math.floor(angle * 3.41 - more * 256))

def checksum(id, length, Write_instruction, GoalPositionAddress, DegL, DegH):
    CHECKSUM = id + length + Write_instruction + GoalPositionAddress + DegL + DegH
    more = int(math.floor(CHECKSUM / 256))
    return 255 - int(math.floor(CHECKSUM - more * 256))

start = [255, 255]
length = 5
Write_instruction = 3
GoalPositionAddress = 30

def write_position(id, angle):
    DegH = GoalPositionValH(angle)
    DegL = GoalPositionValL(angle)
    CHECKSUM = checksum(id, length, Write_instruction, GoalPositionAddress, DegL, DegH)
    print(f"ID {id}: Moving to {angle} degrees with checksum = {CHECKSUM}")
    return "".join(map(chr, start + [id, length, Write_instruction, GoalPositionAddress, DegL, DegH, CHECKSUM]))

def move_servo(id, angle):
    ser.write(write_position(id, angle).encode('latin-1'))
    time.sleep(0.5)  # Increase the delay for slower movement

def control_gripper(id, angle):
    if 140 <= angle <= 180:
        move_servo(id, angle)
    else:
        print(f"Gripper angle {angle} is out of range. Must be between 140 and 180 degrees.")

def handle_trigger(req):
    try:
        # Initialize positions
        control_gripper(1, 180)  # Initialize gripper to 180 degrees
        move_servo(6, 40)  # Initialize joint to 40 degrees
        move_servo(3, 260)  # Initialize base to 260 degrees

        # Move gripper from 180 to 140 degrees
        control_gripper(1, 140)
        # Move gripper back to 180 degrees
        #control_gripper(1, 180)

        # Move joint (ID 6) to 200 degrees, then to 90 degrees
        move_servo(6, 200)
        move_servo(6, 90)

        # Move base (ID 3) from 260 to 210 degrees
        move_servo(3, 210)

        # Return to initial positions
        control_gripper(1, 180)  # Gripper back to 180 degrees
        move_servo(6, 40)  # Joint back to 40 degrees
        move_servo(3, 260)  # Base back to 260 degrees

        # Initialize the speech recognition model
        model = Model(r"/home/leongonn/r1_wiki_ws/src/sr_pkg/src/vosk-model-small-en-us-0.15")
        recognizer = KaldiRecognizer(model, 16000)

        # Initialize the text-to-speech engine
        engine = pyttsx3.init()
        engine.setProperty('rate', 150)  # Lower the value to slow down the speech

        def speak(text):
            engine.say(text)
            engine.runAndWait()

        def listen_for_response():
            mic = pyaudio.PyAudio()
            stream = mic.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=16384)
            stream.start_stream()

            while True:
                try:
                    data = stream.read(2048, exception_on_overflow=False)
                except IOError:
                    print("Overflow occurred, skipping this frame")
                    continue

                if recognizer.AcceptWaveform(data):
                    result = recognizer.Result()
                    text = result[14:-3].lower()
                    print("Recognized:", text)

                    if "yes" in text:
                        return True
                    elif "no" in text:
                        return False
                    else:
                        speak("Please say yes to continue")
                        print("Prompting again: Please say yes to continue")

        # Main script
        speak("I have carried your luggage. Should I follow you now? Please reply yes to continue")

        # Start listening after the TTS output
        response = listen_for_response()

        if response:
            speak("I will follow you now")

        return TriggerResponse(success=True, message="Servo control and speech tasks completed.")

    except Exception as e:
        print("Error:", e)
        return TriggerResponse(success=False, message=str(e))

def control_and_speak_server():
    rospy.init_node('control_and_speak_server')
    s = rospy.Service('control_and_speak', Trigger, handle_trigger)
    print("Ready to control servos and speak.")
    rospy.spin()

if __name__ == "__main__":
    control_and_speak_server()

