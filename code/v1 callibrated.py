import cv2
import mediapipe as mp
import math

mp_face = mp.solutions.face_detection.FaceDetection(model_selection=1, min_detection_confidence=0.1)
cap = cv2.VideoCapture(0)
width = 640
height = 480

def obj_data(img):
    image_input = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = mp_face.process(image_input)
    if not results.detections:
        print("NO FACE")
        return []
    else:
        face_centers = []
        for detection in results.detections:
            bbox = detection.location_data.relative_bounding_box
            x, y, w, h = int(bbox.xmin * width), int(bbox.ymin * height), int(bbox.width * width), int(bbox.height * height)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            center_x = x + w // 2
            center_y = y + h // 2
            face_centers.append((center_x, center_y, w * h))
        return face_centers

def distance(x1, y1, x2, y2):
    return int(math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2))

def pixels_to_meters(area):
    if area <= 0:
        return float('inf')
    return 267.14 * math.pow(area, -0.612)

while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame, (640, 480))

    face_centers = obj_data(frame)
    if face_centers:
        # Find the leftmost and rightmost faces
        leftmost = min(face_centers, key=lambda p: p[0])
        rightmost = max(face_centers, key=lambda p: p[0])

        # Calculate the distance between leftmost and rightmost faces
        left_right_distance = distance(leftmost[0], leftmost[1], rightmost[0], rightmost[1])

        # Ensure left_right_distance is positive and non-zero
        if left_right_distance > 0:
            left_right_distance_meters = pixels_to_meters(left_right_distance)
            cv2.line(frame, (leftmost[0], leftmost[1]), (rightmost[0], rightmost[1]), (255, 0, 0), 2)
            cv2.putText(frame, f"Left-Right Distance: {left_right_distance_meters:.2f} m", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (255, 0, 0), 2)

        # Find the face with the smallest area (farthest person)
        farthest_face = min(face_centers, key=lambda p: p[2])
        farthest_distance_meters = pixels_to_meters(farthest_face[2])

        # Display distance to the farthest person
        cv2.putText(frame, f"Distance to Farthest Face: {farthest_distance_meters:.2f} m", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (255, 0, 0), 2)

    cv2.imshow("FRAME", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
