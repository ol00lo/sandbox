import cv2
import numpy as np
import onnxruntime as ort
import matplotlib.pyplot as plt

class_names = [
    "back_ground", "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
    "trafficlight", "firehydrant", "streetsign", "stopsign", "parkingmeter", "bench", "bird", "cat", "dog",
    "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "hat", "backpack", "umbrella", "shoe",
    "eyeglasses", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sportsball", "kite",
    "baseballbat", "baseballglove", "skateboard", "surfboard", "tennisracket", "bottle", "plate",
    "wineglass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hotdog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
    "mirror", "diningtable", "window", "desk", "toilet", "door", "tvmonitor", "laptop", "mouse", "remote",
    "keyboard", "cellphone", "microwave", "oven", "toaster", "sink", "refrigerator", "blender", "book",
    "clock", "vase", "scissors", "teddybear", "hairdrier", "toothbrush"
]

def draw_boxes(image, boxes, scores, class_ids, need_class=False):
    height, width, _ = image.shape
    for i in range(len(boxes)):
        y_min, x_min, y_max, x_max = boxes[i]
        x_min, x_max, y_min, y_max = int(x_min * width), int(x_max * width), int(y_min * height), int(y_max * height)

        if need_class:
            label = f"{class_names[class_ids[i]]}: {scores[i]:.2f}"
        else:
            label = f"{scores[i]:.2f}"
        cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)
        cv2.putText(image, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

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

def NMS(boxes, scores, class_ids=None, iou_threshold=0.8, ignore_classes=False):
    boxes = np.array(boxes)
    x1 = boxes[:, 1]
    y1 = boxes[:, 0]
    x2 = boxes[:, 3]
    y2 = boxes[:, 2]

    area = (x2 - x1 + 1) * (y2 - y1 + 1)
    ind = np.argsort(scores)[::-1]

    selected_boxes = []
    selected_scores = []
    selected_ids = []

    while len(ind) > 0:
        current_index = ind[0]
        selected_boxes.append(boxes[current_index])
        selected_scores.append(scores[current_index])

        xx1 = np.maximum(x1[current_index], x1[ind[1:]])
        yy1 = np.maximum(y1[current_index], y1[ind[1:]])
        xx2 = np.minimum(x2[current_index], x2[ind[1:]])
        yy2 = np.minimum(y2[current_index], y2[ind[1:]])

        w = np.maximum(0, xx2 - xx1 + 1)
        h = np.maximum(0, yy2 - yy1 + 1)
        overlap = (w * h) / area[ind[1:]]

        if ignore_classes:
            ind = ind[np.where(overlap <= iou_threshold)[0] + 1]
        else:
            class_id_current = class_ids[current_index]
            selected_ids.append(class_id_current)
            remaining_indices = np.where(overlap <= iou_threshold)[0] + 1
            remaining_classes = class_ids[ind[remaining_indices]]
            ind = ind[remaining_indices[remaining_classes != class_id_current]]

    return np.array(selected_boxes), np.array(selected_scores), np.array(selected_ids)

if __name__ == '__main__':
    path = 'model.onnx'

    model = ort.InferenceSession(path)
    image_path = 'pics/spoon2.jpg'
    image = cv2.imread(image_path)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
 
    boxes, scores, class_ids = detect_objects(model, image, threshold=0.1)
    nmsboxes, nmsscores, nmsclass_ids = NMS(boxes, scores, class_ids, iou_threshold=0.7)

    draw_boxes(image, boxes, scores, class_ids, need_class=True)
    plt.imshow(image)
    plt.axis('off')
    plt.show()

    draw_boxes(image, nmsboxes, nmsscores, nmsclass_ids)