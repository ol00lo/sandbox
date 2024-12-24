import cv2
import numpy as np
import imgaug as ia
from imgaug import augmenters as iaa
np.bool = bool

def get_augmenter(level):
    if level == 0:
        return iaa.Sometimes(0.95,  
                iaa.SomeOf((1, None), [
                iaa.Affine(rotate=(-5, 5), mode='reflect'),  
                iaa.Multiply((0.6, 1.2)),
                iaa.OneOf([
                    iaa.GaussianBlur(sigma=(0, 0.7)),  
                    iaa.AverageBlur(k=(2, 5)),
                ]),
                iaa.AddToHue((-50, 50)),
            ], random_order=True))
        
    elif level == 1:
        return iaa.Sometimes(0.95, 
            iaa.SomeOf((2, None), [
                iaa.Affine(rotate=(-5, 5), mode='reflect'), 
                iaa.Multiply((0.6, 1.5)),  
                iaa.AddToHue((-50, 50)),
                iaa.AdditiveGaussianNoise(scale=(0, 0.1200)),
                iaa.MotionBlur(k=(3, 5)),  
                iaa.JpegCompression(compression=(80, 99))], 
            random_order=True))
    else:
        raise ValueError(":((")


def display_images(data, data_lables, img_height, img_width):
    images = data
    labels = data_lables

    image = np.zeros((2 * img_height, 2 * img_width, 3), dtype=np.uint8)
    for idx, img in enumerate(images):
        row = idx // 2
        col = idx % 2
        image[row * img_height:(row + 1) * img_width, col * img_height:(col + 1) * img_width] = img

    cv2.imshow('Press esc to continue', image)

    for label in data_lables:
        print(f"Label: {label}");

    key = cv2.waitKey(0)
    if key == 27:
        cv2.destroyAllWindows()
        return
