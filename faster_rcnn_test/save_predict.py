import os
import json
import numpy as np
import onnxruntime as ort
import cv2
import detect_onnx
from tqdm import tqdm 

session = ort.InferenceSession("model.onnx")
image_folder = "val2017/val2017"  
output_json = "predictions11.json"

predictions = []

image_files = os.listdir(image_folder)

for image_file in tqdm(image_files, desc="Processing images", unit="image"):
    image_path = os.path.join(image_folder, image_file)
    image = cv2.imread(image_path)

    if image is None:
        print(f"Warning: Could not read image {image_file}. Skipping.")
        continue
         
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    boxes, scores, class_ids = detect_onnx.detect_objects(session, image)
    
    # like in annotation/instances_val2017.json
    for i in range(len(boxes)):
        boxes[i][0] *= image.shape[0]  
        boxes[i][1] *= image.shape[1]  
        boxes[i][2] *= image.shape[0]  
        boxes[i][3] *= image.shape[1]
        
    for box, clss, score in zip(boxes, class_ids, scores):
        predictions.append({
        "image_id": (image_file.split('.')[0]).lstrip('0'),
        "bbox": box.tolist(),
        "score": score.item(),
        "category_id": int(clss)
        })

with open(output_json, "w") as f:
    json.dump(predictions, f)
print(f"Predictions saved to {output_json}.")