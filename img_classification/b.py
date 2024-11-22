import numpy as np
from PIL import Image
import tensorflow as tf
import os
from tqdm import tqdm

def preprocess_image(image_path):
    img = Image.open(image_path)
    img = img.resize((32, 32))
    img_array = np.array(img)
    img_array = img_array / 255.0
    img_array = np.expand_dims(img_array, axis=0)
    return img_array


def print_metrics(cm, y_true, y_pred, n_classes):
    accuracy = np.trace(cm) / np.sum(cm)  
    precision = np.diagonal(cm) / np.sum(cm, axis=0, where=np.sum(cm, axis=0) != 0)  
    recall = np.diagonal(cm) / np.sum(cm, axis=1, where=np.sum(cm, axis=1) != 0)  
    f1 = 2 * (precision * recall) / (precision + recall + 1e-10)
    mean_average_precision = sum(precision) / n_classes
    mean_average_recall = sum(recall) / n_classes
    formatted_precision = [f"{p:.3f}" for p in precision]
    formatted_recall = [f"{r:.3f}" for r in recall]
    formatted_f1 = [f"{f:.3f}" for f in f1]
    print(f"Accuracy: {accuracy:.3f}\n\n"
          f"Precision: {', '.join(formatted_precision)}\n\n"
          f"Recall: {', '.join(formatted_recall)}\n\n"
          f"F1 Score: {', '.join(formatted_f1)}\n\n"
          f"Mean Average Precision: {mean_average_precision:.3f}\n\n"
          f"Mean Average Recall: {mean_average_recall:.3f}")

images_folder = 'my_pics'
model_path = 'best_model1.keras'
model = tf.keras.models.load_model(model_path)
batch_size = 10

class_names = ['airplane', 'automobile', 'bird', 'cat', 'deer',
               'dog', 'frog', 'horse', 'ship', 'truck']


y_true, y_pred = [], []
class_folders = [f.path for f in os.scandir(images_folder) if f.is_dir()]
for class_index, class_folder in enumerate(class_folders):
    images_list = os.listdir(class_folder)
    for i in tqdm(range(0, len(images_list), batch_size)):
        batch_images = images_list[i:i + batch_size]
        for filename in batch_images:
            image_path = os.path.join(class_folder, filename)
            preprocessed_image = preprocess_image(image_path)
            predicted_class_index = np.argmax(model.predict(preprocessed_image))
            y_true.append(class_index)
            y_pred.append(predicted_class_index)

n_classes = len(class_names)

confusion_matrix = np.zeros((n_classes, n_classes), dtype=int)
for true, pred in zip(y_true, y_pred):
    confusion_matrix[true][pred] += 1

print("Confusion Matrix:")
print(confusion_matrix)
print("\nMetrics:")
print(class_names)
print_metrics(confusion_matrix, y_true, y_pred, n_classes)