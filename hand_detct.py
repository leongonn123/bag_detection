#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
import cv2
import mediapipe as mp
import numpy as np
import threading

# Initialize ROS node
rospy.init_node('hand_detection_publisher', anonymous=True)

# Setup ROS publisher for hand landmark count
landmark_pub = rospy.Publisher('/visualization_markers', Int32, queue_size=10)

# Global variables for image data
global_image_rgb = None  # Store image in RGB format
image_lock = threading.Lock()
emergency_stop = threading.Event()

# Initialize mediapipe hand detection
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

def rgb_image_callback(msg):
    global global_image_rgb

    try:
        # Check the image encoding and handle accordingly
        if msg.encoding == 'mono8':  # Single-channel grayscale image
            rgb_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_GRAY2RGB)  # Convert grayscale to RGB
        elif msg.encoding == 'rgb8':  # RGB image
            rgb_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        elif msg.encoding == 'bayer_grbg8':  # Bayer GRBG pattern image
            bayer_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            rgb_image = cv2.cvtColor(bayer_image, cv2.COLOR_BAYER_GR2RGB)  # Convert Bayer to RGB
        else:
            rospy.logerr(f"Unsupported image encoding: {msg.encoding}")
            return

        with image_lock:
            global_image_rgb = rgb_image

    except Exception as e:
        rospy.logerr(f"Error processing RGB image: {e}")

def process_and_publish_landmarks():
    global global_image_rgb

    with mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
        while not rospy.is_shutdown():
            if emergency_stop.is_set():
                break

            with image_lock:
                if global_image_rgb is not None:
                    # Prepare RGB image for MediaPipe processing
                    image_rgb = global_image_rgb.copy()
                    image_rgb.flags.writeable = False
                    results = hands.process(image_rgb)
                    image_rgb.flags.writeable = True

                    if results.multi_hand_landmarks:
                        for hand_landmarks in results.multi_hand_landmarks:
                            # Draw landmarks and connections for each detected hand
                            mp_drawing.draw_landmarks(
                                image_rgb,
                                hand_landmarks,
                                mp_hands.HAND_CONNECTIONS,
                                mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                                mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=2, circle_radius=2)
                            )

                            # Check if the hand is pointing
                            is_pointing = check_if_hand_is_pointing(hand_landmarks)

                            # Publish the result (1 for pointing, 0 for not)
                            landmark_pub.publish(1 if is_pointing else 0)

                            # Display status
                            cv2.putText(
                                image_rgb, f'Hand Pointing: {"Yes" if is_pointing else "No"}',
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (0, 255, 0) if is_pointing else (0, 0, 255), 2, cv2.LINE_AA
                            )

                            # Optionally highlight specific landmarks if pointing
                            if is_pointing:
                                index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                                cv2.circle(image_rgb, (int(index_tip.x * image_rgb.shape[1]), int(index_tip.y * image_rgb.shape[0])), 10, (255, 255, 0), -1)

                    # Show the RGB image with landmarks
                    cv2.imshow('RGB Image with Hand Detection', image_rgb)

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        rospy.signal_shutdown("Emergency stop initiated by user.")
                        emergency_stop.set()
                        break

def check_if_hand_is_pointing(hand_landmarks):
    thumb_tip = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
    index_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
    middle_finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
    ring_finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
    pinky_tip = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]

    # Check if the index finger is extended
    # Allow some flexibility for variations in hand position
    index_extended = index_tip.y < wrist.y - 0.03 and abs(index_tip.x - wrist.x) < 0.1
    print(f'Index Extended: {index_extended} (Index Tip: {index_tip}, Wrist: {wrist})')

    # Check if other fingers are curled
    other_fingers_curled = all([
        middle_finger_tip.y > wrist.y + 0.1,
        ring_finger_tip.y > wrist.y + 0.1,
        pinky_tip.y > wrist.y + 0.1
    ])
    print(f'Other Fingers Curled: {other_fingers_curled} (Middle: {middle_finger_tip}, Ring: {ring_finger_tip}, Pinky: {pinky_tip})')

    # Check if the index finger is pointing straight
    # Allow some flexibility for variations in hand orientation
    index_pointing_straight = abs(index_tip.x - thumb_tip.x) < 0.15
    print(f'Index Pointing Straight: {index_pointing_straight} (Index Tip: {index_tip}, Thumb Tip: {thumb_tip})')

    # Return True if the index finger is extended, other fingers are curled, and the index finger is pointing straight
    return index_extended and other_fingers_curled and index_pointing_straight


# Subscribe to the RGB image topic
rospy.Subscriber('/camera/rgb/image_raw', Image, rgb_image_callback)

# Start the processing and publishing thread
landmark_thread = threading.Thread(target=process_and_publish_landmarks)
landmark_thread.start()

# Spin to keep the script alive
rospy.spin()

# Wait for the landmark thread to finish
landmark_thread.join()

cv2.destroyAllWindows()
