import os
import numpy as np
import cv2
import time
import random
import tensorflow as tf
from tensorflow.keras.applications import MobileNet
from tensorflow.keras import layers, models, callbacks
import augmenter

IMG_HEIGHT, IMG_WIDTH = 224, 224
BATCH_SIZE = 32
EPOCHS = 10  
DATA_DIR = 'c:\\Users\\mymri\\repos\\clock_sett\\'

def make_set(DATA_DIR):
    import pandas as pd
    all_data = []
    for clock_dir in os.listdir(DATA_DIR):
        clock_path = os.path.join(DATA_DIR, clock_dir)

        if os.path.isdir(clock_path):
            csv_file = os.path.join(clock_path, 'dataset_info.csv')

            if os.path.isfile(csv_file):
                df = pd.read_csv(csv_file)
                for index, row in df.iterrows():
                    image_name = row['imagepath']
                    minutes = row['minutes']
                    image_path = os.path.join(clock_path, image_name)
                    all_data.append((image_path, minutes))
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
                if need_shuffle:
                    random.shuffle(data)  
                data_copy = data.copy() 
                continue 

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
            batch_data = np.array(batch_data, dtype=np.uint8) 
            batch_labels = np.array(batch_labels) 

            if aug_sequence is not None:
                batch_data = aug_sequence(images=batch_data)  
                augmenter.display_images(batch_data[:4], batch_labels[:4], 224, 224)
            
            yield batch_data, batch_labels  

def custom_loss(y_true, y_pred):
    y_true_deg = tf.convert_to_tensor(y_true * 6, dtype=tf.float32)
    y_pred_deg = tf.convert_to_tensor(y_pred * 6, dtype=tf.float32)

    y_true_deg = tf.math.floormod(y_true_deg, 4320)
    y_pred_deg = tf.math.floormod(y_pred_deg, 4320)

    delta_deg = y_pred_deg - y_true_deg

    delta_deg = (delta_deg + 2160) % 4320 - 2160

    return tf.reduce_mean(tf.square(delta_deg))

train_data, valid_data = make_set(DATA_DIR)

aug_sequence = lambda x: augmenter.get_augmenter(x)

train_generator = data_generator(train_data, BATCH_SIZE, aug_sequence(0), True)
valid_generator = data_generator(valid_data, BATCH_SIZE, lambda x: x, False)

base_model = MobileNet(weights='imagenet', include_top=False, input_shape=(IMG_HEIGHT, IMG_WIDTH, 3))
base_model.trainable = False

model = models.Sequential([
    base_model,
    layers.GlobalAveragePooling2D(),
    layers.Dense(128, activation='relu'),
    layers.Dropout(0.5), 
    layers.Dense(1, activation='linear')  
])

true = np.array([0.0, 720.0])
pred = np.array([720.0, 0.0])
print(f"0 = {custom_loss(true, pred)}")
true = np.array([1.0])
pred = np.array([719.0])
print(f"144.0 = {custom_loss(true, pred)}")
true = np.array([710])
pred = np.array([350.0])
print(f"a lot = {custom_loss(true, pred)}")
true = np.array([1.0])
pred = np.array([2.0])
print(f"36.0 = {custom_loss(true, pred):.4f}")

model.compile(optimizer='adam', loss=custom_loss)
log_dir = os.path.join("logs", "fit", f"model_run_{int(time.time())}")
tensorboard_callback = tf.keras.callbacks.TensorBoard(log_dir=log_dir, histogram_freq=1)
model_checkpoint = callbacks.ModelCheckpoint(filepath='best_model.keras', save_best_only=True, monitor='val_loss')

model.fit(
    train_generator,
    steps_per_epoch=len(train_data) // BATCH_SIZE,
    validation_data=valid_generator ,
    validation_steps=len(valid_data) // BATCH_SIZE,
    epochs=EPOCHS,
    callbacks=[tensorboard_callback, model_checkpoint]
)
