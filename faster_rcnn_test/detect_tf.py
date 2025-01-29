import cv2
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
import kagglehub
import common_func as cf

path = kagglehub.model_download("tensorflow/faster-rcnn-inception-resnet-v2/tensorFlow2/1024x1024")
#path = 'C:\\Users\\mymri\\.cache\\kagglehub\\models\\tensorflow\\faster-rcnn-inception-resnet-v2\\tensorFlow2\\1024x1024\\1'

def detect_objects(model,image, threshold=0.5):
    image = image.astype(np.uint8)  
    input_tensor = tf.convert_to_tensor(image)  
    input_tensor = tf.expand_dims(input_tensor, axis=0)  
    detections = model(input_tensor)

    boxes = detections['detection_boxes'][0].numpy()
    scores = detections['detection_scores'][0].numpy()
    class_ids = detections['detection_classes'][0].numpy().astype(int)

    valid_indices = np.where(scores > threshold)[0]
    boxes = boxes[valid_indices]
    scores = scores[valid_indices]
    class_ids = class_ids[valid_indices]

    return boxes, scores, class_ids

if __name__ == '__main__':
    model = tf.saved_model.load(path)
    #image_path = 'pics/cow2.jpg'
    image_path = 'pics/spoon2.jpg'
    image = cv2.imread(image_path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    boxes, scores, class_ids = detect_objects(model, image, threshold=0.1)
    nmsboxes, nmsscores, nmsclass_ids = cf.NMS(boxes, scores, class_ids, iou_threshold=0.7)

    #draw_boxes(image, boxes, scores, class_ids, class_names, need_class = True)
    cf.draw_boxes(image, nmsboxes, nmsscores, nmsclass_ids)
    plt.imshow(image)
    plt.axis('off')
    plt.show()