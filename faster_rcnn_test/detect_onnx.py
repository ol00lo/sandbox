import cv2
import numpy as np
import onnxruntime as ort
import matplotlib.pyplot as plt
import common_func as cf

def detect_objects(session, image, threshold=0.5):
    image = np.expand_dims(image, axis=0) 
    input_name = session.get_inputs()[0].name
    detections = session.run(None, {input_name: image})
    
    boxes = detections[1][0]  
    scores = detections[4][0] 
    class_ids = detections[2][0].astype(int)  
    
    valid_indices = np.where(scores > threshold)[0]
    boxes = boxes[valid_indices]
    scores = scores[valid_indices]
    class_ids = class_ids[valid_indices]

    return boxes, scores, class_ids


if __name__ == '__main__':
    path = 'model.onnx'

    model = ort.InferenceSession(path)
    image_path = 'pics/spoon2.jpg'
    image = cv2.imread(image_path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
 
    boxes, scores, class_ids = detect_objects(model, image, threshold=0.1)
    nmsboxes, nmsscores, nmsclass_ids = cf.NMS(boxes, scores, class_ids, iou_threshold=0.8)

    #draw_boxes(image, boxes, scores, class_ids, need_class=True)
    cf.draw_boxes(image, nmsboxes, nmsscores, nmsclass_ids)
    plt.imshow(image)
    plt.axis('off')
    plt.show()
