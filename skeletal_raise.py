#!/usr/bin/env python3

import cv2
import mediapipe as mp

# Initialize MediaPipe Pose and Hands modules
mp_pose = mp.solutions.pose
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Set up the camera capture
cap = cv2.VideoCapture(0)

# Define a function to check if the hand is raised above the face
def is_hand_above_face(pose_landmarks, hand_landmarks):
    if pose_landmarks and hand_landmarks:
        nose = pose_landmarks[mp_pose.PoseLandmark.NOSE].y
        left_hand = hand_landmarks[mp_hands.HandLandmark.WRIST].y
        right_hand = hand_landmarks[mp_hands.HandLandmark.WRIST].y
        
        # Check if either hand is above the nose
        if left_hand < nose or right_hand < nose:
            return True
    return False

# Initialize MediaPipe Pose and Hands
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose, \
     mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
    
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            break

        # Flip the image horizontally for a later selfie-view display
        frame = cv2.flip(frame, 1)

        # Convert the BGR image to RGB
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process the image and find pose and hand landmarks
        results_pose = pose.process(image_rgb)
        results_hands = hands.process(image_rgb)

        # Draw landmarks and connections on the frame
        if results_pose.pose_landmarks:
            mp_drawing.draw_landmarks(frame, results_pose.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        
        if results_hands.multi_hand_landmarks:
            for hand_landmarks in results_hands.multi_hand_landmarks:
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # Check if the hand is raised above the face
                if is_hand_above_face(results_pose.pose_landmarks.landmark, hand_landmarks.landmark):
                    print("Hand is raised: True")
                else:
                    print("Hand is raised: False")

        # Show the frame
        cv2.imshow('Hand Above Face Detection', frame)

        # Press 'q' to exit the loop
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

# Release resources
cap.release()
cv2.destroyAllWindows()
