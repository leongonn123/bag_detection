#!/usr/bin/env python3

import cv2
from ultralytics import YOLO

# Load the trained model
model = YOLO("/home/leongonn/r1_wiki_ws/src/skeletal_cam/src/bag_detection/best.pt")  # Replace with your model's path

# Initialize the USB camera (usually index 0)
cap = cv2.VideoCapture(0)

# Check if the camera is opened correctly
if not cap.isOpened():
    print("Error: Could not open video stream from USB camera.")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Error: Failed to capture image.")
        break

    # Use the model to detect objects in the frame
    results = model(frame)

    # Get the results
    detections = results[0]
    
    # Filter detections based on confidence threshold
    filtered_boxes = []
    filtered_scores = []
    filtered_classes = []

    for det in detections.boxes:
        if det.conf > 0.9:  # Check if confidence is greater than 0.9
            filtered_boxes.append(det.xyxy.tolist())  # Append bounding box coordinates
            filtered_scores.append(det.conf.tolist())  # Append confidence scores
            filtered_classes.append(det.cls.tolist())  # Append class IDs

    # Print the number of bags detected and their confidence levels
    num_bags = len(filtered_boxes)
    print(f'Number of bags detected: {num_bags}')
    
    if num_bags > 0:
        for i, score in enumerate(filtered_scores):
            print(f'Bag {i + 1} confidence level: {score[0]:.2f}')

    # Create an annotated frame with filtered detections
    annotated_frame = frame.copy()
    for box, score, cls in zip(filtered_boxes, filtered_scores, filtered_classes):
        # Draw bounding box with confidence score on the frame
        x1, y1, x2, y2 = map(int, box[0])  # Convert box coordinates to integer
        label = "Bag"  # Label for detected objects
        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Draw bounding box
        cv2.putText(annotated_frame, f'{label} Conf: {score[0]:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)  # Display label and confidence score
    
    # Display the resulting frame
    cv2.imshow('Bag Detection', annotated_frame)

    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture and close the window
cap.release()
cv2.destroyAllWindows()
