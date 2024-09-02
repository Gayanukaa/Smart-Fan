import cv2
import mediapipe as mp
import math
import RPi.GPIO as GPIO
from time import sleep
import math

servo_pin = 11
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 50)  # frequency 50Hz
pwm.start(0)

left_duty_cycle = 2.5
neutral_duty_cycle = 7.5
right_duty_cycle = 12.5

mp_face = mp.solutions.face_detection.FaceDetection(model_selection=1, min_detection_confidence=0.5)
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
    return int(math.sqrt(((y1 - y2) ** 2) + ((x1 - x2) ** 2)))

def pixels_to_meters(area):
    if area <= 0:
        return float('inf')
    return 267.14 * math.pow(area, -0.612)

def set_servo_angle(duty_cycle):
    pwm.ChangeDutyCycle(duty_cycle)
    sleep(0.05)

def setAngle(angle):
    for angle in range(-angle, (angle+1), 10):
        duty_cycle = neutral_duty_cycle + angle / 180 * (right_duty_cycle - left_duty_cycle)
        set_servo_angle(duty_cycle)
    for angle in range(angle, -(angle+1), -10):
        duty_cycle = neutral_duty_cycle + angle / 180 * (right_duty_cycle - left_duty_cycle)
        set_servo_angle(duty_cycle)

while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame, (640, 480))

    cv2.line(frame, (width // 2, 0), (width // 2, height), (255, 0, 0), 2)

    face_centers = obj_data(frame)
    if face_centers:
        for i in range(len(face_centers)):
            for j in range(i + 1, len(face_centers)):
                dist = distance(face_centers[i][0], face_centers[i][1], face_centers[j][0], face_centers[j][1])
                cv2.line(frame, (face_centers[i][0], face_centers[i][1]), (face_centers[j][0], face_centers[j][1]), (0, 255, 0), 2)

        leftmost = min(face_centers, key=lambda p: p[0])
        rightmost = max(face_centers, key=lambda p: p[0])

        left_right_distance = distance(leftmost[0], leftmost[1], rightmost[0], rightmost[1])

        farthest_face = min(face_centers, key=lambda p: p[2])
        farthest_distance_meters = pixels_to_meters(farthest_face[2])

        cv2.putText(frame, f"Distance to Farthest Face: {farthest_distance_meters:.2f} m", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        if left_right_distance > 0:
            left_right_distance_meters = pixels_to_meters(left_right_distance)
            cv2.line(frame, (leftmost[0], leftmost[1]), (rightmost[0], rightmost[1]), (255, 0, 0), 2)
            cv2.putText(frame, f"Left to Right Distance: {left_right_distance_meters:.2f} m", (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            face_range = rightmost[0] - leftmost[0]
            if face_range > 0:
                angle = abs(-90 + (180 * (rightmost[0] - leftmost[0]) / width))
                print(angle)
                if(angle<30): setAngle(30)
                elif(angle<60): setAngle(60)
                else: setAngle(90)
                cv2.putText(frame, f"Servo Angle: {angle:.2f} degrees", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

    else:
        setAngle(90)

    cv2.imshow("FRAME", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
pwm.stop()
GPIO.cleanup()
