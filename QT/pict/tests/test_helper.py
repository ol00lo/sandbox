import os
import cv2
import numpy as np

def create_test_folder():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    folder = current_dir + "/test_images/"
    os.makedirs(folder, exist_ok=True)

    for i in range(10):
        image = np.zeros((100, 100, 3), dtype=np.uint8)
        image[:, :, 0] = 255 * i / 10
        image[:, :, 1] = 255 * (10 - i) / 10
        image[:, :, 2] = i
        cv2.imwrite(folder + f"test{i}.jpg", image)

    return folder

if __name__ == "__main__":
    create_test_folder()
    print("DONE")