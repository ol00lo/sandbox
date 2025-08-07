import os
import cv2
import shutil
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

def create_image(path):
    try:
        dir_path = os.path.normpath(path) + os.sep
        os.makedirs(dir_path, exist_ok=True)
        file_path = os.path.join(dir_path, "test.jpg")
        image = np.ones((100, 100, 3), dtype=np.uint8) * 255
        if cv2.imwrite(file_path, image):
            return file_path
        return None

    except Exception as e:
        print(f"Ошибка при создании изображения: {e}")
        return None

def clear_folder(folder):
    for filename in os.listdir(folder):
        file_path = os.path.join(folder, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print('Failed to delete %s. Reason: %s' % (file_path, e))

if __name__ == "__main__":
    create_test_folder()
    print("DONE")