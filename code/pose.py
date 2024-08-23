import cv2
import mediapipe as mp
from gpiozero import PWMLED
import math

led = PWMLED(14)
mp_pose = mp.solutions.pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
cap = cv2.VideoCapture(0)
width = 640
height = 480
led.value = 0

def obj_data(img):
    image_input = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = mp_pose.process(image_input)
    if not results.pose_landmarks:
        print("NO PERSON DETECTED")
        return []
    else:
        body_centers = []
        for lm in results.pose_landmarks.landmark:
            x, y = int(lm.x * width), int(lm.y * height)
            body_centers.append((x, y))
        return body_centers

def distance(x1, y1, x2, y2):
    return int(math.sqrt(((y1 - y2) ** 2) + ((x1 - x2) ** 2)))

def pixels_to_meters(distance_pixels):
    # Ensure distance_pixels is positive and non-zero
    if distance_pixels <= 0:
        return float('inf')  # Return infinity or some large value to indicate invalid distance
    return 267.14 * math.pow(distance_pixels, -0.612)

while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame, (640, 480))
    
    # Draw imaginary centerline
    cv2.line(frame, (width // 2, 0), (width // 2, height), (255, 0, 0), 2)
    
    body_centers = obj_data(frame)
    if body_centers:
        # Calculate the distances between all pairs of detected humans (based on keypoints)
        for i in range(len(body_centers)):
            for j in range(i + 1, len(body_centers)):
                dist = distance(body_centers[i][0], body_centers[i][1], body_centers[j][0], body_centers[j][1])
                cv2.line(frame, (body_centers[i][0], body_centers[i][1]), (body_centers[j][0], body_centers[j][1]), (0, 255, 0), 2)
        
        # Find the leftmost and rightmost keypoints (to measure width of human figure)
        leftmost = min(body_centers, key=lambda p: p[0])
        rightmost = max(body_centers, key=lambda p: p[0])
        
        # Calculate the distance between leftmost and rightmost keypoints
        left_right_distance = distance(leftmost[0], leftmost[1], rightmost[0], rightmost[1])
        
        # Ensure left_right_distance is positive and non-zero
        if left_right_distance > 0:
            left_right_distance_meters = pixels_to_meters(left_right_distance)
            cv2.line(frame, (leftmost[0], leftmost[1]), (rightmost[0], rightmost[1]), (255, 0, 0), 2)
            cv2.putText(frame, f"Left-Right Distance: {left_right_distance_meters:.2f} meters", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (255, 0, 0), 2)
        
        # Display distances of keypoints from centerline
        for center_x, center_y in body_centers:
            x_distance = abs(center_x - width // 2)
            cv2.putText(frame, f"X Distance from Center: {x_distance} pixels", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (255, 0, 0), 2)
    
    cv2.imshow("FRAME", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
