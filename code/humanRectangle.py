import cv2
import numpy as np

# Load pre-trained HOG detector for human detection
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

def calculate_distance(pixel_width, focal_length, known_width):
    # Formula for distance calculation: distance = (known_width * focal_length) / pixel_width
    return (known_width * focal_length) / pixel_width

# Initialize camera
cap = cv2.VideoCapture(0)

# Known width of the object (person) in inches
KNOWN_WIDTH = 20  # Assumed width of an average human shoulder width in inches

# Focal length (calculated experimentally or obtained from camera specifications) in pixels
FOCAL_LENGTH = 600  # Example value, should be adjusted based on your setup

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    # Resize frame for faster processing (optional, depends on camera resolution and processing power)
    frame = cv2.resize(frame, (640, 480))
    
    # Detect humans in the frame
    (humans, _) = hog.detectMultiScale(frame, winStride=(4, 4), padding=(8, 8), scale=1.05)
    
    # Iterate over detected humans and calculate distance to each
    for (x, y, w, h) in humans:
        # Draw bounding box around detected human
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Calculate distance to human
        distance = calculate_distance(w, FOCAL_LENGTH, KNOWN_WIDTH)
        
        # Display distance above bounding box
        cv2.putText(frame, f'{distance:.2f} inches', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    # Display the resulting frame
    cv2.imshow('Frame', frame)
    
    # Exit on 'q' press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close any open windows
cap.release()
cv2.destroyAllWindows()
