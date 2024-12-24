import numpy as np
import matplotlib.pyplot as plt
import tensorflow as tf
from keras import datasets as data
import os

checkpoint = tf.keras.callbacks.ModelCheckpoint(
    'model_epoch{epoch:02d}_accuracy_{val_accuracy:.5f}.keras',
    monitor='val_accuracy',
    save_best_only=True,
    mode='max',
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

inputs = tf.keras.Input(shape=(32, 32, 3))
x = tf.keras.layers.Conv2D(16, (3, 3), activation='relu', padding='same')(inputs)
x = tf.keras.layers.MaxPool2D((2, 2))(x)
x = tf.keras.layers.Conv2D(32, (3, 3), activation='relu', padding='same')(x)
x = tf.keras.layers.MaxPool2D((2, 2))(x)
x = tf.keras.layers.Conv2D(64, (3, 3), activation='relu', padding='same')(x)
x = tf.keras.layers.MaxPool2D((2, 2))(x)
x = tf.keras.layers.Conv2D(16, (3, 3), activation='relu', padding='same')(x)
x = tf.keras.layers.MaxPool2D((2, 2))(x)
#-----------------------------------
# x = tf.keras.layers.Conv2D(32, (3, 3), activation='relu', padding='same')(x)
# x = tf.keras.layers.MaxPool2D((2, 2))(x)
#-----------------------------------
x = tf.keras.layers.Conv2D(128, (3, 3), activation='relu', padding='same')(x)
x = tf.keras.layers.Flatten()(x)
outputs = tf.keras.layers.Dense(10, activation='softmax')(x)
model = tf.keras.Model(inputs=inputs, outputs=outputs)

model.summary()
model.compile(optimizer='adam',
              loss='categorical_crossentropy',
              metrics=['accuracy'])

model.fit(X_train, y_train, batch_size=32, epochs=20, validation_data=(X_test, y_test), callbacks=checkpoint)