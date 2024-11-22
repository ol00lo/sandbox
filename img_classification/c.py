import os
import numpy as np
from PIL import Image
import tensorflow as tf
from tqdm import tqdm

def preprocess_image(image_path):
    img = Image.open(image_path).resize((32, 32)) 
    img_array = np.array(img) / 255.0
    return np.expand_dims(img_array, axis=0)

images_folder = 'my'  
model_path = 'best_model1.keras'  
model = tf.keras.models.load_model(model_path)

class_names = ['airplane', 'automobile', 'bird', 'cat', 'deer',
               'dog', 'frog', 'horse', 'ship', 'truck']

class_counts = {class_name: 0 for class_name in class_names}
correct_counts = {class_name: 0 for class_name in class_names}

for filename in tqdm(os.listdir(images_folder)):
    if filename.endswith(('.jpeg', '.jpg', '.png', '.webp')):  
        true_class = filename.split('_')[0]  
        
        image_path = os.path.join(images_folder, filename)
        preprocessed_image = preprocess_image(image_path)

        predicted_class_index = np.argmax(model.predict(preprocessed_image))
        predicted_class = class_names[predicted_class_index]

        class_counts[true_class] += 1
        if true_class == predicted_class:
            correct_counts[true_class] += 1

print("\nResult:")
for class_name in class_names:
    print(f"{class_name}: Total: {class_counts[class_name]}, Correct: {correct_counts[class_name]}")