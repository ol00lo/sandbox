import tensorflow as tf
from tensorflow.keras.layers import Input, Lambda
from tensorflow.keras.models import Model, load_model
import numpy as np
import cv2

old_model_path = "best_model1.keras"
confidence = 0.7
test_img_path = "cat.jpeg"

def get_class_index_with_confidence_threshold(inputs):
    x, threshold = inputs
    max_confidence = tf.reduce_max(x, axis=-1)
    class_index = tf.argmax(x, axis=-1)
    condition = tf.greater_equal(max_confidence, threshold)
    return tf.where(condition, class_index, tf.constant(-1, dtype=class_index.dtype))

def preprocess_layer(x):
    x = tf.image.resize(x, (32, 32), method=tf.image.ResizeMethod.BILINEAR)
    x = tf.cast(x, tf.float32)
    x = x[..., ::-1]
    return x

loaded_model = load_model(old_model_path)

confidence_threshold = Input(shape=(1,))
inp = Input(shape=(240, 320, 3), dtype=tf.uint8)

x = Lambda(preprocess_layer)(inp)

for layer in loaded_model.layers[0:]:
    if isinstance(layer, tf.keras.layers.Layer): 
        x = layer(x)

class_index_output = Lambda(get_class_index_with_confidence_threshold)([x, confidence_threshold])

modified_model = Model(inputs=[inp, confidence_threshold], outputs=[class_index_output])

modified_model.set_weights(loaded_model.get_weights())

    
for i in range(len(modified_model.layers)):
    if modified_model.layers[i].name == "conv2d":
        weights, bias = modified_model.layers[i].get_weights()
        weights = weights / 255.0
        modified_model.layers[i].set_weights([weights, bias])
        print(f"{i} layer modified")
        break

modified_model.save("modified_model.keras")

def new_preprocess(image_path):
    img = cv2.imread(image_path)
    img = cv2.resize(img, (320, 240), interpolation=cv2.INTER_LINEAR)
    img_array = np.array(img)
    img_array = np.expand_dims(img_array, axis=0)
    return img_array

def old_preprocess(image_path):    
    img = cv2.imread(image_path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (320, 240), interpolation=cv2.INTER_LINEAR)
    img = cv2.resize(img, (32,32), interpolation=cv2.INTER_LINEAR)
    img_array = np.array(img, dtype=np.float32)
    img_array = img_array / 255.0
    img_array = np.expand_dims(img_array, axis=0)
    return img_array

input_image = new_preprocess(test_img_path)
input_value = np.array([[confidence]]) 
predicted_class_index = modified_model.predict([input_image, input_value])
print(f"For new model predicted class index: {predicted_class_index}")

loaded_model = load_model(old_model_path)
input_image = old_preprocess(test_img_path)
predict = loaded_model.predict(input_image)
predicted_class_index = np.argmax(predict, axis=-1)
confidence = np.max(predict, axis=-1)
print(f"For old model predicted class index: {predicted_class_index} with confidence: {confidence}")