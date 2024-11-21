import numpy as np
from PIL import Image
import tensorflow as tf
import os

def preprocess_image(image_path):
    img = Image.open(image_path)
    img = img.resize((32, 32))
    img_array = np.array(img)
    img_array = img_array / 255.0
    img_array = np.expand_dims(img_array, axis=0)
    return img_array

images_folder = 'my_pics'
model_path = 'new_best_model1.keras'
model = tf.keras.models.load_model(model_path)

class_names = ['airplane', 'automobile', 'bird', 'cat', 'deer', 
               'dog', 'frog', 'horse', 'ship', 'truck']

for filename in os.listdir(images_folder):
    if filename.endswith('.jpeg') or filename.endswith('.jpg') or filename.endswith('.png') or filename.endswith('.webp'): 
        image_path = os.path.join(images_folder, filename)  
        preprocessed_image = preprocess_image(image_path) 
        predictions = model.predict(preprocessed_image)
        predicted_class_name = class_names[np.argmax(predictions)]
        print(f"Изображение: {filename} - класс: {predicted_class_name}")