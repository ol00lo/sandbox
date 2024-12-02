import argparse
import os
import glob
import random
import time
import cv2
import numpy as np
import imgaug as ia
from imgaug import augmenters as iaa
np.bool = bool

def get_augmenter(level):
    if level == 0:
        return iaa.Sequential([
            iaa.Sometimes(0.05, iaa.Noop()), 
            iaa.SomeOf((1, None), [
                iaa.Fliplr(0.5),
                iaa.Affine(rotate=(-20, 20), mode='reflect'),  
                iaa.Multiply((0.6, 1.2)),
                iaa.OneOf([
                    iaa.GaussianBlur(sigma=(0, 0.7)),  
                    iaa.AverageBlur(k=(2, 5)),
                ]),
                iaa.AddToHue((-50, 50)),
            ], random_order=True),
        ])
        
    elif level == 1:
        return iaa.Sequential([
            iaa.Sometimes(0.05, iaa.Noop()), 
            iaa.SomeOf((2, 5), [
                iaa.Fliplr(0.5),
                iaa.Affine(rotate=(-20, 20), mode='reflect'), 
                iaa.Multiply((0.6, 1.5)),  
                iaa.AddToHue((-50, 50)),
                iaa.AdditiveGaussianNoise(scale=(0, 0.1200)),
                iaa.MotionBlur(k=(3, 5)),  
                iaa.JpegCompression(compression=(80, 99)),  
                iaa.Rain(speed=(0.1, 0.3))
            ], random_order=True)
        ])
    elif level == 2:
        return iaa.Sequential([
            iaa.Sometimes(0.05, iaa.Noop()), 
            iaa.SomeOf((3, 7), [
                iaa.Fliplr(0.5),
                iaa.Affine(rotate=(-20, 20), mode='reflect'),
                iaa.Multiply((0.6, 1.5)),
                iaa.AddToHue((-50, 50)),
                iaa.AdditiveGaussianNoise(scale=(0, 0.1*200)),
                iaa.MotionBlur(k=(3, 5)),
                iaa.PiecewiseAffine(scale=(0.01, 0.05)),
                iaa.JpegCompression(compression=(80, 99)),  
                iaa.CropAndPad(percent=(-0.15, 0.15), pad_mode=ia.ALL),
                iaa.CoarseDropout((0.1, 0.2), size_percent=(0.02, 0.06), per_channel=0.5),
                iaa.GammaContrast(gamma=(0.5, 2.0)),
                iaa.imgcorruptlike.Snow(severity=2)
            ], random_order=True)
        ])
    else:
        raise ValueError(":((")

def prog(image_files, nbatch, level, img_size):
    selected_files = random.sample(image_files, min(nbatch, len(image_files)))
    images = []
    load_times = []

    for file in selected_files:
        start_time = time.time()
        img = cv2.imread(file)

        images.append(img)  
        load_time = time.time() - start_time
        load_times.append(load_time)
    augmenter = get_augmenter(level)
    images_aug = augmenter(images=images) 
    images_aug = [cv2.resize(img, (img_size, img_size)) for img in images_aug]
    
    return selected_files, images_aug, load_times

def display_images(selected_files, images, labels, img_size):
    images = images[:4]
    if len(images) == 1:
        image = images[0]
    elif len(images) == 2:
        image = np.zeros((img_size, 2*img_size, 3), dtype=np.uint8) 
        image[:, :img_size] = images[0]
        image[:, img_size:] = images[1] 
    else:
        image = np.zeros((2 * img_size, 2 * img_size, 3), dtype=np.uint8)
        for idx, img in enumerate(images):
            row = idx // 2
            col = idx % 2
            image[row * img_size:(row + 1) * img_size, col * img_size:(col + 1) * img_size] = img

    cv2.imshow('SPACE - next; ESC - exit', image)

    for filename, label in zip(selected_files, labels):
        print(f"Filename: {filename.ljust(54, ' ')}  Label: {label}");

    while True:
        key = cv2.waitKey(0)
        if key == ord(' '):  
            break
        elif key == 27:
            cv2.destroyAllWindows()
            return True


# PARSE
parser = argparse.ArgumentParser()
parser.add_argument('--datadir', type=str, required=True, help='Directory with images and annotations')
parser.add_argument('--nbatch', type=int, default=4, help='Batch size (default: 4)')
parser.add_argument('--level', type=int, choices=[0, 1, 2], default=0, help='Augmentation level (0, 1, 2)')
parser.add_argument('--img_size', type=int, default=128, help='Size for resizing images (default: 128)') 
args = parser.parse_args()


#PROG 
datadir = args.datadir
nbatch = args.nbatch
level = args.level
#datadir = "c:/Users/mymri/repos/my_pics/bird"
img_size = args.img_size
#nbatch = 4
#level = 2
#img_size = 128

image_files = glob.glob(os.path.join(datadir, '*.jpeg')) + \
            glob.glob(os.path.join(datadir, '*.png')) + glob.glob(os.path.join(datadir, '*.jpg')) 

epoch = 0

while True:
    epoch += 1
    print(f"Epoch: {epoch}")
    random.shuffle(image_files)
    
    total_img_load_time = 0
    img_count = 0

    for i in range(0, len(image_files), nbatch):
        batch_files = image_files[i:i + nbatch]
        if not batch_files:
            break
        labels = [os.path.basename(file).split('_')[0] for file in batch_files] 
        start_time_load = time.time()
        selected_files, images_aug, load_times = prog(batch_files, nbatch, level, img_size)
        end_time_load = time.time()

        total_img_load_time += sum(load_times)
        img_count += len(load_times)

        display_exit = display_images(selected_files, images_aug, labels, img_size)
        if display_exit: 
            elapsed_time = (total_img_load_time) / img_count * 1000
            print(f"END... Average image loading time: {elapsed_time:.4f} milliseconds.")
            exit(0)

        print(f"Batch loading time: {end_time_load - start_time_load:.4f} seconds")
