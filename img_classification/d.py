import tensorflow as tf
from tensorflow.keras.layers import Input, Lambda
from tensorflow.keras.models import Model, load_model
from PIL import Image
import numpy as np

def classify_with_threshold_class_index(inputs):
    x, threshold = inputs
    max_confidence = tf.reduce_max(x, axis=-1)
    class_index = tf.argmax(x, axis=-1)
    condition = tf.greater_equal(max_confidence, threshold)
    return tf.where(condition, class_index, tf.constant(-1, dtype=class_index.dtype))

def classify_with_threshold_confidence(inputs):
    x, threshold = inputs
    max_confidence = tf.reduce_max(x, axis=-1)
    condition = tf.greater_equal(max_confidence, threshold)
    return tf.where(condition, max_confidence, tf.zeros_like(max_confidence))

def output_shape(input_shapes):
    return input_shapes[0][:-1] + (1,)

loaded_model = load_model("model_epoch14_accuracy_0.71740.keras")

inp = Input(shape=(320, 240, 3))
confidence_threshold = Input(shape=(1,), name='confidence_threshold')
x = tf.keras.layers.Resizing(32, 32)(inp)

for layer in loaded_model.layers[1:]:
    layer.trainable = False  
    if isinstance(layer, tf.keras.layers.Layer): 
        x = layer(x)

class_index_output = Lambda(classify_with_threshold_class_index, output_shape=output_shape)([x, confidence_threshold])
confidence_output = Lambda(classify_with_threshold_confidence, output_shape=output_shape)([x, confidence_threshold])

modified_model = Model(inputs=[inp, confidence_threshold], outputs=[class_index_output, confidence_output])
modified_model.set_weights(loaded_model.get_weights())
loaded_model.summary()
modified_model.summary()
def preprocess_image(image_path):
    img = Image.open(image_path)
    img = img.resize((240, 320))
    img_array = np.array(img).astype(np.float32)
    img_array = img_array / 255.0
    img_array = np.expand_dims(img_array, axis=0)
    return img_array

preprocessed_image = preprocess_image("cat.jpeg")
input_value = np.array([[0.7]]) 
predicted_class_index, predicted_confidence = modified_model.predict([preprocessed_image, input_value])
print(f"Predicted Class Index: {predicted_class_index}")
print(f"Predicted Confidence: {predicted_confidence}")

def preprocess(image_path):    
    img = Image.open(image_path)
    img = img.resize((32, 32))
    img_array = np.array(img)
    img_array = img_array / 255.0
    img_array = np.expand_dims(img_array, axis=0)
    return img_array

img = preprocess("cat.jpeg")
predict = loaded_model.predict(img)
predicted_class_index = np.argmax(predict, axis=-1)
confidence = np.max(predict, axis=-1)
print(f"Predicted Class Index: {predicted_class_index}")
print(f"Predicted Confidence: {confidence}")