import os
import cv2
import time

interval = 5      #seconds
output_dir = "photos"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("Error: Failed to open the camera.")
    exit()

try:
    while True:
        # pass buffering
        for _ in range(10):
            ret, frame = cap.read()     
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = os.path.join(output_dir, f"photo_{timestamp}.jpg")  

        cv2.imwrite(filename, frame)
        print(f"Photo saved: {filename}")
        
        time.sleep(interval)
except KeyboardInterrupt:
    print("Program Completion...")
finally:
    cap.release()
    cv2.destroyAllWindows()
