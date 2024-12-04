import os
import cv2
import time

interval = 5      #seconds
output_dir = "photos"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Failed to open the camera.")
    exit()

try:
    while True:
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