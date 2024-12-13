import os
import numpy as np
import tensorflow as tf
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.applications import MobileNet
from tensorflow.keras import layers, models, callbacks

IMG_HEIGHT, IMG_WIDTH = 224, 224
BATCH_SIZE = 32
EPOCHS = 10  
DATA_DIR = 'output'

train_datagen = ImageDataGenerator(
    rescale=1.0/255.0,
    width_shift_range=0.2,
    height_shift_range=0.2,
    shear_range=0.2,
    zoom_range=0.2,
    fill_mode='nearest'
)

train_generator = train_datagen.flow_from_directory(
    DATA_DIR,
    target_size=(IMG_HEIGHT, IMG_WIDTH),
    batch_size=BATCH_SIZE,
    class_mode=None
)

base_model = MobileNet(weights='imagenet', include_top=False, input_shape=(IMG_HEIGHT, IMG_WIDTH, 3))
base_model.trainable = False

model = models.Sequential([
    base_model,
    layers.GlobalAveragePooling2D(),
    layers.Dense(128, activation='relu'),
    layers.Dropout(0.5), 
    layers.Dense(train_generator.num_classes, activation='softmax')  
])

def custom_loss(y_true, y_pred):
    y_pred = tf.math.floormod(y_pred, 720)  
    y_true = tf.math.floormod(y_true, 720)  
    diff = tf.abs(y_true - y_pred)
    loss = tf.reduce_mean(tf.minimum(diff, 720 - diff))
    return loss

model.compile(optimizer='adam', loss=custom_loss, metrics=['accuracy'])

log_dir = "logs/fit/" + tf.datetime.now().strftime("%Y%m%d-%H%M%S")
tensorboard_callback = callbacks.TensorBoard(log_dir=log_dir, histogram_freq=1)

model_checkpoint = callbacks.ModelCheckpoint(filepath='best_model.keras', save_best_only=True, monitor='val_accuracy')

model.fit(
    train_generator,
    steps_per_epoch=train_generator.samples // BATCH_SIZE,
    epochs=EPOCHS,
    callbacks=[tensorboard_callback, model_checkpoint]
)