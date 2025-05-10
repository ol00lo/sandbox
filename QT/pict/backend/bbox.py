from typing import Dict, List, Tuple
from pathlib import Path
from PyQt6 import QtCore

class BBoxList:
    def __init__(self, output_file: str = "bbox_output.csv") -> None:
        self.output_file: str = output_file
        self.bbox_data: Dict[str, List[Tuple[str, QtCore.QRect]]] = {}

        with open(self.output_file, 'w') as f:
            f.write("Path,n_boxes,Label,x1,y1,x2,y2\n")

    
    def add_bbox(self, bbox: QtCore.QRect, label: str, image_path: str):
        if image_path not in self.bbox_data:
            self.bbox_data[image_path] = []
        self.bbox_data[image_path].append((label, bbox))

        with open(self.output_file, 'a') as f:
            n_boxes = len(self.bbox_data[image_path])
            line = (
                f'"{image_path}",{n_boxes},"{label}",'
                f'{bbox.left()},{bbox.top()},'
                f'{bbox.right()},{bbox.bottom()}\n'
            )
            f.write(line)

        print(f"BBox appended to: {Path(self.output_file).absolute()}")

    def get_bboxes_for_image(self, path):
        if path in self.bbox_data:
            return self.bbox_data[path]
        else:
            return []

    def clear_bboxes_for_image(self, path):
        if path in self.bbox_data:
            del self.bbox_data[path]

if __name__ == "__main__":
    bbox_list = BBoxList()

    rect1 = QtCore.QRect(10, 20, 30, 40)
    rect2 = QtCore.QRect(50, 60, 70, 80)
    rect3 = QtCore.QRect(15, 25, 35, 45)

    bbox_list.add_bbox(rect1, "cat", "images/img1.jpg")
    bbox_list.add_bbox(rect2, "dog", "images/img1.jpg")
    bbox_list.add_bbox(rect3, "cat", "images/img2.jpg")