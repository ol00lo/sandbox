import os
import numpy as np
import cv2
import random
import math
import shutil
import tensorflow as tf
from tensorflow.keras.applications import MobileNet
from tensorflow.keras import layers, models, callbacks
import augmenter
import custom_loss

seed_value = 0
random.seed(seed_value)
np.random.seed(seed_value)

IMG_HEIGHT, IMG_WIDTH = 224, 224
BATCH_SIZE = 32
EPOCHS = 1000  
DATA_DIR = 'c:\\Users\\mymri\\repos\\clock_sett\\'
AUG_LEVEL = 0
DELTA_ACCURACY = 5

def make_set(data_dir):
    import pandas as pd
    all_data = []
    for clock_dir in os.listdir(data_dir):
        clock_path = os.path.join(data_dir, clock_dir)

        if os.path.isdir(clock_path):
            csv_file = os.path.join(clock_path, 'dataset_info.csv')

            if os.path.isfile(csv_file):
                df = pd.read_csv(csv_file)
                for index, row in df.iterrows():
                    image_name = row['imagepath']
                    minutes = row['minutes']
                    image_path = os.path.join(clock_path, image_name)
                    all_data.append((image_path, minutes))

    all_data.sort(key=lambda x: x[0])
    np.random.shuffle(all_data)

    train_size = int(0.9 * len(all_data))
    train_data = all_data[:train_size]
    valid_data = all_data[train_size:]

    print("Train data size :", len(train_data))
    print("Validation data size :", len(valid_data))
    return train_data, valid_data   

def data_generator(data, batch_size, aug_sequence=None, need_shuffle=True):
    data_copy = data.copy()
    if need_shuffle:
        random.shuffle(data_copy) 
    
    while True:
        batch_data = []
        batch_labels = []
        for _ in range(batch_size):
            if len(data_copy) == 0: 
                data_copy = data.copy() 
                if need_shuffle:
                    random.shuffle(data_copy)  
                break 

            filename, label = data_copy.pop(0)  
            image = cv2.imread(filename)
            if image is None:
                print(f"Warning: Could not read image: {filename}")
                continue  

            image = cv2.resize(image, (224,224))
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) 
            batch_data.append(image)  
            batch_labels.append(label)  
        if batch_data:  
            batch_data = np.array(batch_data) 
            batch_labels = np.array(batch_labels) 
            if aug_sequence is not None:
                batch_data = aug_sequence(images=batch_data)
                # augmenter.display_images(batch_data[:4], batch_labels[:4], 224, 224)

            # normalize from [0,255] to [-1,1]
            batch_data = tf.keras.applications.mobilenet.preprocess_input(batch_data)
            yield batch_data, batch_labels  


def custom_accuracy(y_true, y_pred):
    delta = DELTA_ACCURACY
    lower_bound = y_pred - delta
    upper_bound = y_pred + delta
    y_true = tf.cast(y_true, tf.float32)
    return tf.reduce_mean(tf.cast((y_true >= lower_bound) & (y_true <= upper_bound), tf.float32))

train_data, valid_data = make_set(DATA_DIR)

aug_sequence = augmenter.get_augmenter(AUG_LEVEL)

train_generator = data_generator(train_data, BATCH_SIZE, aug_sequence, True)
valid_generator = data_generator(valid_data, BATCH_SIZE)

base_model = MobileNet(weights='imagenet', include_top=False, input_shape=(IMG_HEIGHT, IMG_WIDTH, 3))
base_model.trainable = False

model = models.Sequential([
    base_model,
    layers.GlobalAveragePooling2D(),
    layers.Dense(128, activation='relu'),
    layers.Dropout(0.5), 
    layers.Dense(1, activation='linear')  
])

model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=1e-4),
              loss=custom_loss.custom_loss, metrics=[custom_accuracy]) 

log_dir = os.path.join("logs", "fit", "model_run")
if os.path.exists(log_dir):
    shutil.rmtree(log_dir)  
os.makedirs(log_dir) 
tensorboard_callback = tf.keras.callbacks.TensorBoard(log_dir=log_dir, histogram_freq=1)

model_checkpoint = callbacks.ModelCheckpoint(
    filepath='model_epoch{epoch:02d}_val_custom_accuracy_{val_custom_accuracy:.5f}.keras',
    save_best_only=True,
    monitor='val_custom_accuracy',
    mode='max',
    verbose=1
)

model.fit(
    train_generator,
    steps_per_epoch=math.ceil(len(train_data) / BATCH_SIZE),
    validation_data=valid_generator,
    validation_steps=math.ceil(len(valid_data) / BATCH_SIZE),
    epochs=EPOCHS,
    callbacks=[tensorboard_callback, model_checkpoint]
)

model.save("final_model.keras")