import cv2

# Load pre-trained Haar Cascade classifier for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def calculate_distance(pixel_width, focal_length, known_width):
    # Formula for distance calculation: distance = (known_width * focal_length) / pixel_width
    # Convert inches to meters (1 inch = 0.0254 meters)
    return (known_width * focal_length) / (pixel_width * 0.0254)

# Initialize camera
cap = cv2.VideoCapture(0)

# Known width of the object (face) in meters
KNOWN_WIDTH = 0.19  # Assumed width of an average human face in meters

# Focal length (calculated experimentally or obtained from camera specifications) in pixels
FOCAL_LENGTH = 630  # Example value, should be adjusted based on your setup

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    # Convert frame to grayscale for face detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect faces in the frame
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    
    # Iterate over detected faces and calculate distance to each
    for (x, y, w, h) in faces:
        # Draw bounding box around detected face
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # Calculate distance to face
        distance = calculate_distance(w, FOCAL_LENGTH, KNOWN_WIDTH)
        
        # Display distance above bounding box
        cv2.putText(frame, f'{distance:.2f} meters', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    
    # Display the resulting frame
    cv2.imshow('Frame', frame)
    
    # Exit on 'q' press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close any open windows
cap.release()
cv2.destroyAllWindows()
