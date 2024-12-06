import tensorflow as tf
from tensorflow.keras.layers import Input, Conv2D, MaxPooling2D, Flatten, Dense, Layer,Lambda
from tensorflow.keras.models import Model, load_model
import numpy as np

loaded_model = load_model("model_epoch01_accuracy_0.52120.keras")
inp = Input(shape=(320, 240, 3))
confidence_threshold = Input(shape=(1,))

x = tf.keras.layers.MaxPooling2D(pool_size =(4, 3), strides=(4, 3))(inp)
x = tf.keras.layers.AveragePooling2D(pool_size=(3, 3), strides=(2, 2))(x)
x = tf.keras.layers.AveragePooling2D(pool_size=(8, 8), strides=(1, 1))(x) 
#x = tf.keras.layers.Resizing(32, 32)(x) 
# ===============old
x = tf.keras.layers.Conv2D(16, (3, 3), activation='relu', padding='same')(x)
x = tf.keras.layers.MaxPool2D((2, 2))(x)
x = tf.keras.layers.Conv2D(32, (3, 3), activation='relu', padding='same')(x)
x = tf.keras.layers.MaxPool2D((2, 2))(x)
x = tf.keras.layers.Conv2D(64, (3, 3), activation='relu', padding='same')(x)
x = tf.keras.layers.MaxPool2D((2, 2))(x)
x = tf.keras.layers.Conv2D(16, (3, 3), activation='relu', padding='same')(x)
x = tf.keras.layers.MaxPool2D((2, 2))(x)
x = tf.keras.layers.Conv2D(128, (3, 3), activation='relu', padding='same')(x)
x = tf.keras.layers.Flatten()(x)
x = Dense(10, activation='softmax')(x)
# ===============old

def classify_with_threshold(args):
    probabilities, threshold = args 
    class_index = tf.argmax(probabilities, axis=-1)  
    confidence = tf.reduce_max(probabilities, axis=-1)  
    return tf.where(confidence >= threshold, class_index, -1) 

x = Lambda(classify_with_threshold)([x, confidence_threshold])

modified_model = Model(inputs=[inp, confidence_threshold], outputs=x)
modified_model.set_weights(loaded_model.get_weights())
loaded_model.summary()
modified_model.summary()
modified_model.save("modified_model.keras")