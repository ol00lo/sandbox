import argparse
import os
import glob
import random
import time
import cv2
import numpy as np
import imgaug as ia
from imgaug import augmenters as iaa

def get_augmenter(level):
    if level == 0:
        return iaa.Sequential([
            iaa.Fliplr(0.5),
            iaa.Affine(rotate=(-20, 20)),
            iaa.Multiply((0.6, 1.5))  
        ])
    elif level == 1:
        return iaa.Sequential([
            iaa.Fliplr(0.5),
            iaa.Affine(rotate=(-20, 20)),
            iaa.Multiply((0.6, 1.5)),  
            iaa.AddToHue((-50, 50)),
            iaa.AdditiveGaussianNoise(scale=(0, 0.1*200))
        ])
    elif level == 2:
        return iaa.Sequential([
            iaa.Fliplr(0.5),
            iaa.Affine(rotate=(-20, 20)),
            iaa.Multiply((0.6, 1.5)),  
            iaa.AddToHue((-50, 50)),
            iaa.AdditiveGaussianNoise(scale=(0, 0.1*200)),
            iaa.PiecewiseAffine(scale=(0.01, 0.05)),
            iaa.CropAndPad(percent=(-0.25, 0.25))
        ])
    else:
        raise ValueError(":((")

def prog(image_files, nbatch, level):
    selected_files = random.sample(image_files, min(nbatch, len(image_files)))
    images = []

    for file in selected_files:
        img = cv2.imread(file)  
        img = cv2.resize(img, (128, 128))  
        images.append(img)
        
    augmenter = get_augmenter(level)
    images_aug = augmenter(images=np.array(images))  
    return selected_files, images_aug


def display_images(selected_files, images):
    images = images[:4]
    if len(images) == 1:
        image = images[0]
    elif len(images) == 2:
        image = np.zeros((128, 256, 3), dtype=np.uint8) 
        image[:, :128] = images[0]
        image[:, 128:] = images[1] 
    else:
        image = np.zeros((2 * 128, 2 * 128, 3), dtype=np.uint8)
        for idx, img in enumerate(images):
            row = idx // 2
            col = idx % 2
            image[row * 128:(row + 1) * 128, col * 128:(col + 1) * 128] = img

    cv2.imshow('SPACE - next; ESC - exit', image)

    for filename, label in zip(selected_files, {1,2,3,4}):
        print(f"Filename: {filename.ljust(54, ' ')}  Label: {label}");

    while True:
        key = cv2.waitKey(0)
        if key == ord(' '):  
            break
        elif key == 27:
            cv2.destroyAllWindows()
            exit(0)


# PARSE
parser = argparse.ArgumentParser()
parser.add_argument('--datadir', type=str, required=True, help='Directory with images and annotations')
parser.add_argument('--nbatch', type=int, default=4, help='Batch size (default: 4)')
parser.add_argument('--level', type=int, choices=[0, 1, 2], default=0, help='Augmentation level (0, 1, 2)')
args = parser.parse_args()


#PROG 
datadir = args.datadir
nbatch = args.nbatch
level = args.level
#datadir = "c:/Users/mymri/repos/my_pics/bird"
#nbatch = 4
#level = 2

image_files = glob.glob(os.path.join(datadir, '*.jpeg'))  
epoch = 0

while True:
    epoch += 1
    print(f"Epoch: {epoch}")
    random.shuffle(image_files)
    
    for i in range(0, len(image_files), nbatch):
        batch_files = image_files[i:i + nbatch]
        if not batch_files:
            break

        start_time_load = time.time()
        selected_files, images_aug = prog(batch_files,nbatch, level)
        end_time_load = time.time()
        display_images(selected_files, images_aug)
        load_time = end_time_load - start_time_load
        print(f"Batch loading time: {load_time:.4f} seconds")
