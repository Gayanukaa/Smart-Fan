import cv2

face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def calculate_distance(pixel_width, focal_length, known_width):
    return (known_width * focal_length) / (pixel_width * 0.0254)

cap = cv2.VideoCapture(0)

KNOWN_WIDTH = 0.19  # Assumed width
FOCAL_LENGTH = 630  # Example value

while True:
    ret, frame = cap.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        distance = calculate_distance(w, FOCAL_LENGTH, KNOWN_WIDTH)
        cv2.putText(frame, f'{distance:.2f} meters', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow('Frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
