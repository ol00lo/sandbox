import numpy as np
import matplotlib.pyplot as plt
import tensorflow as tf
from keras import datasets as data
import os

checkpoint = tf.keras.callbacks.ModelCheckpoint(
    'model_epoch{epoch:02d}_loss_{loss:.5f}.keras', 
    monitor='val_loss',  
    save_best_only=True,  
    mode='min',  
    verbose=1  
)

to_dir = "cifar10"
if not os.path.exists(to_dir):  
    os.makedirs(to_dir)
    (X_train, y_train), (X_test, y_test) = data.cifar10.load_data()
    np.save(os.path.join(to_dir, "X_train.npy"), X_train)
    np.save(os.path.join(to_dir, "y_train.npy"), y_train)
    np.save(os.path.join(to_dir, "X_test.npy"), X_test)
    np.save(os.path.join(to_dir, "y_test.npy"), y_test)

X_train = np.load(os.path.join(to_dir, "X_train.npy"))
y_train = np.load(os.path.join(to_dir, "y_train.npy"))
X_test = np.load(os.path.join(to_dir, "X_test.npy"))
y_test = np.load(os.path.join(to_dir, "y_test.npy"))

X_train = X_train.astype('float32') / 255.0
X_test = X_test.astype('float32') / 255.0

y_train = tf.keras.utils.to_categorical(y_train, num_classes=10)
y_test = tf.keras.utils.to_categorical(y_test, num_classes=10)

model = tf.keras.models.Sequential([
    tf.keras.layers.InputLayer(input_shape=(32, 32, 3)),
    tf.keras.layers.Conv2D(16, (3, 3), activation='relu', padding='same'),
    tf.keras.layers.MaxPool2D((2, 2)),
    tf.keras.layers.Conv2D(32, (3, 3), activation='relu', padding='same'),  
    tf.keras.layers.MaxPool2D((2, 2)),
    tf.keras.layers.Conv2D(64, (3, 3), activation='relu', padding='same'),
    tf.keras.layers.MaxPool2D((2, 2)),
    tf.keras.layers.Conv2D(16, (3, 3), activation='relu', padding='same'),  
    tf.keras.layers.MaxPool2D((2, 2)),
    #-----------------------------------
    #tf.keras.layers.Conv2D(32, (3, 3), activation='relu', padding='same'), 
    #tf.keras.layers.MaxPool2D((2, 2)),
    #-----------------------------------
    tf.keras.layers.Conv2D(128, (3, 3), activation='relu', padding='same'),  
    tf.keras.layers.Flatten(),
    tf.keras.layers.Dense(10, activation='softmax')
])

model.summary()
model.compile(optimizer='adam',
              loss='categorical_crossentropy',
              metrics=['accuracy'])

model.fit(X_train, y_train, batch_size=32, epochs=20, validation_data=(X_test, y_test), callbacks=checkpoint)