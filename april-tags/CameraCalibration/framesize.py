import cv2

cap = cv2.VideoCapture(0)  # 0 indicates the default camera
cap.set(3, 640)  # Set the width (frame size)
cap.set(4, 480)  # Set the height (frame size)

# You can now capture frames from the camera with the specified frame size
while True:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow("Camera Feed", frame)

# Release the camera and close the OpenCV windows when done
cap.release()
cv2.destroyAllWindows()
