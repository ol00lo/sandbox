import json
import numpy as np
from common_func import class_names

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

    if(len(pred_class) == 0 and len(ann_class) == 0):
        return None
    
    pred_class.sort(key=lambda x: x['score'], reverse=True)

    true_positives = []
    false_positives = []
    
    matched_annotations = []

    for pred in pred_class:
        pred_bbox = pred['bbox']
        matched = False
        with_image = [a for a in ann_class if a['image_id'] == int(pred['image_id'])]

        for annotation in with_image:
            annotation_bbox = annotation['bbox']
            if annotation['id'] not in matched_annotations and iou(pred_bbox, annotation_bbox) >= iou_threshold:
                matched = True
                matched_annotations.append(annotation['id'])
                break

        true_positives.append(int(matched))
        false_positives.append(int(not matched))

    tp_cumsum = np.cumsum(true_positives)
    fp_cumsum = np.cumsum(false_positives)

    precision = tp_cumsum / (tp_cumsum + fp_cumsum + np.finfo(float).eps)
    recall = tp_cumsum / len(ann_class)
    precision = np.maximum.accumulate(precision[::-1])[::-1]
    
    if recall[0] != 0:
        recall = np.insert(recall, 0, 0)
        precision = np.insert(precision, 0, precision[0])

    if recall[-1] != 1:
        recall = np.append(recall, recall[-1])
        precision = np.append(precision, 0)

    ap = np.trapz(precision, recall)
    return ap

def compute_map(predictions, annotations, classes, iou_threshold=0.5):
    ap_per_class = {}
    nc = 1
    for c in classes:
        ap = compute_ap(predictions, annotations, nc, iou_threshold)
        if(ap is not None):
            ap_per_class[nc] = ap
        else:
            print(f"Class {c} is not present in predictions and annotations")
        nc += 1
    mAP = np.mean(list(ap_per_class.values()))
    return ap_per_class, mAP


if __name__ == "__main__":
    predict_json = "predictions.json"  
    with open(predict_json) as f:
        predictions = json.load(f)

    with open('annotations/instances_val2017.json') as f:
        annotations = json.load(f)
    annotations = annotations['annotations']

    ap_per_class, mappp = compute_map(predictions, annotations, class_names, iou_threshold=0.5)

    for i, ap in ap_per_class.items():
        print(f'AP for class {i}: {ap*100:.2f} %')

    print(f'AP: {mappp*100:.3f} %')