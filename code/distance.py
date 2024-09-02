import cv2
import mediapipe as mp
import time
from gpiozero import PWMLED


led = PWMLED(14)
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
        led.value = 0
        #time.sleep(0.5)
        return None
    else:
        for detection in results.detections:
            bbox = detection.location_data.relative_bounding_box
            x, y, w, h = int(bbox.xmin * width), int(bbox.ymin * height), int(bbox.width * width), int(
                bbox.height * height)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            led.value = 1
            return w * h  # Return the area of the bounding box as a rough estimate of distance


def pixels_to_meters(area):
    return 100000/area


while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame, (640, 480))
    obj_size = obj_data(frame)
    if obj_size is not None:
        distance_meters = pixels_to_meters(obj_size)
        print(f"Distance to Face: {distance_meters:.2f} meters")
        cv2.putText(frame, f"Distance to Face: {distance_meters:.2f} meters", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (255, 0, 0), 2)
    cv2.imshow("FRAME", frame)
    #time.sleep(0.5)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
