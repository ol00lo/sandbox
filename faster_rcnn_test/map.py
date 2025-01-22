import json
import numpy as np

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

predict_json = "predictions.json"  

def iou(boxA, boxB):
    x1 = boxB[0]
    y1 = boxB[1]
    x2 = boxB[0] + boxB[2]
    y2 = boxB[1] + boxB[3]
    xA = max(boxA[1], x1)
    yA = max(boxA[0], y1)
    xB = min(boxA[3], x2)
    yB = min(boxA[2], y2)
    
    interArea = max(0, xB - xA) * max(0, yB - yA)
    
    boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxBArea = (x2 - x1) * (y2 - y1)
    
    iou_value = interArea / float(boxAArea + boxBArea - interArea + 1e-6)  
    return iou_value

def compute_ap(predictions, annotations, class_id, iou_threshold=0.5):
    pred_class = [p for p in predictions if p['category_id'] == class_id]
    ann_class = [a for a in annotations if a['category_id'] == class_id]
    print(f"Number of predictions {class_id}: {len(pred_class)}")
    print(f"Number of annotations {class_id}: {len(ann_class)}\n")
    
    true_positives = []
    false_positives = []
    
    for pred in pred_class:
        pred_bbox = pred['bbox']
        matched = False
        with_image = [a for a in ann_class if a['image_id'] == int(pred['image_id'])]

        for annotation in with_image:
            annotation_bbox = annotation['bbox']
            if iou(pred_bbox, annotation_bbox) >= iou_threshold:
                matched = True
                break
        
        true_positives.append(int(matched))
        false_positives.append(int(not matched))
    
    tp_cumsum = np.cumsum(true_positives)
    fp_cumsum = np.cumsum(false_positives)
    
    precision = tp_cumsum / (tp_cumsum + fp_cumsum + np.finfo(float).eps)  
    recall = tp_cumsum / len(ann_class) 
    
    ap = np.sum((recall[1:] - recall[:-1]) * precision[1:])
    #ap = np.trapz(precision, recall)  
    return ap

def compute_map(predictions, annotations, classes, iou_threshold=0.5):
    ap_per_class = {}
    nc = 1
    for _ in classes:
        if nc in {12, 26, 29, 30, 45, 66, 68, 69, 71, 83, 91}:
            nc += 1
            continue
        ap = compute_ap(predictions, annotations, nc, iou_threshold)
        ap_per_class[nc] = ap
        nc += 1
    mAP = np.mean(list(ap_per_class.values()))
    return ap_per_class, mAP


with open(predict_json) as f:
    predictions = json.load(f)

with open('annotations/instances_val2017.json') as f:
    annotations = json.load(f)
annotations = annotations['annotations']

ap_per_class, mappp = compute_map(predictions, annotations, class_names, iou_threshold=0.5)

for i, ap in ap_per_class.items():
    print(f'AP for class {i}: {ap*100:.2f} %')

print(f'AP: {mappp*100:.3f} %')