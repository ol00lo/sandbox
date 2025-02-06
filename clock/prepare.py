import os
import numpy as np
import cv2
import tqdm


def prepare_data(data_list, output_dir, N, aug_sequence=None):
    os.makedirs(output_dir, exist_ok=True)
    
    for i in tqdm.tqdm(range(0, len(data_list), N)):
        batch_data = []
        batch_labels = []
        
        for j in range(i, min(i + N, len(data_list))):
            image_path, label = data_list[j]
            image = cv2.imread(image_path)
            if image is None:
                print(f"Warning: Could not read image: {image_path}")
                continue
            
            image = cv2.resize(image, (224, 224))
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            batch_data.append(image)
            batch_labels.append(label)
        
        if aug_sequence is not None:
            batch_data = aug_sequence(images=np.array(batch_data))

        np.save(os.path.join(output_dir, f"data_{i//N}.npy"), np.array(batch_data))
        np.save(os.path.join(output_dir, f"labels_{i//N}.npy"), np.array(batch_labels))
