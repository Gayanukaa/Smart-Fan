import cv2
import mediapipe as mp
from gpiozero import PWMLED
import math

led = PWMLED(15)
mp_face = mp.solutions.face_detection.FaceDetection(model_selection=1, min_detection_confidence=0.5)
cap = cv2.VideoCapture(0)
width = 640
height = 480
led.value = 0

def obj_data(img):
    image_input = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = mp_face.process(image_input)
    if not results.detections:
        print("NO FACE")
        led.value = 0.0
        return None, None  # Return None for both width*height and x-coordinate
    else:
        for detection in results.detections:
            bbox = detection.location_data.relative_bounding_box
            x, y, w, h = int(bbox.xmin * width), int(bbox.ymin * height), int(bbox.width * width), int(bbox.height * height)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # Calculate the x-coordinate of the center of the face
            center_x = x + w // 2
            return w * h, center_x

def pixels_to_meters(area):
    return 267.14 * math.pow(area, -0.612)

while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame, (640, 480))
    
    # Draw imaginary centerline
    cv2.line(frame, (width // 2, 0), (width // 2, height), (255, 0, 0), 2)
    
    obj_size, center_x = obj_data(frame)
    if obj_size is not None:
        distance_meters = pixels_to_meters(obj_size)
        if center_x is not None:
            # Set LED brightness based on the distance to the face
            if distance_meters < 1:
                led.value = 0.2
            elif 1 <= distance_meters <= 1.5:
                led.value = 0.5
            elif distance_meters > 2:
                led.value = 1
            else:
                led.value = 0.05  # Default value when no condition is met

            # Display distance and x distance from the centerline
            x_distance = abs(center_x - width // 2)
            print(f"Distance to Face: {distance_meters:.2f} meters, X Distance from Center: {x_distance} pixels")
            cv2.putText(frame, f"Distance to Face: {distance_meters:.2f} meters", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (255, 0, 0), 2)
            cv2.putText(frame, f"X Distance from Center: {x_distance} pixels", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (255, 0, 0), 2)
    
    cv2.imshow("FRAME", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
