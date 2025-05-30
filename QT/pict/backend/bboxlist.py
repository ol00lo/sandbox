from typing import Dict, List, Tuple
from pathlib import Path
from PyQt6 import QtCore
import os
from backend.box import Box

class BBoxList:
    def __init__(self, folder: str = "", output_file: str = "") -> None:
        if not output_file == "":
            abs_folder = os.path.abspath(folder)
            self.output_file = os.path.join(abs_folder, output_file)
            self.bbox_data: Dict[str, List[Box]] = {}
            
            csv_files = [f for f in os.listdir(os.path.abspath(folder)) if f.lower().endswith('.csv')]
            if csv_files:
                self.output_file = os.path.join(abs_folder, csv_files[0])
            else:
                with open(self.output_file, 'w') as f:
                    f.write("Path,n_boxes,Label,x1,y1,x2,y2\n")

    def add_bbox(self, bbox: Box, name):
        image_name = name

        if image_name not in self.bbox_data:
            self.bbox_data[image_name] = []

        self.bbox_data[image_name].append(bbox)

        with open(self.output_file, 'a') as f:
            n_boxes = len(self.bbox_data[image_name])
            line = (
                f'"{image_name}",{n_boxes},"{bbox.label}",'
                f'{bbox.left()},{bbox.top()},'
                f'{bbox.left() + bbox.width()},{bbox.top() + bbox.height()}\n'
            )
            f.write(line)

        print(f"BBox appended to: {Path(self.output_file).absolute()}")

    def delete_bbox(self, box: Box, name):
        if name not in self.bbox_data:
            return False

        for c in self.bbox_data[name]:
            if box.label == c.label and rectangles_are_similar(box, c):
                self.bbox_data[name].remove(c)
                self._write_bbox_data_to_file()
                print(f"BBox deleted from: {Path(self.output_file).absolute()}")
                return True

        return False

    def _write_bbox_data_to_file(self):
        with open(self.output_file, 'w') as f:
            f.write("Path,n_boxes,Label,x1,y1,x2,y2\n")
            for bbox_name, bboxes in self.bbox_data.items():
                n_boxes = len(bboxes)
                for bbox in bboxes:
                    line = (
                        f'"{bbox_name}",{n_boxes},"{bbox.label}",'
                        f'{bbox.left()},{bbox.top()},'
                        f'{bbox.right()},{bbox.bottom()}\n'
                    )
                    f.write(line)

    def update_bbox(self, old_box: Box, new_box: Box, name):
        self.delete_bbox(old_box, name)
        new_box.label = old_box.label
        self.add_bbox(new_box, name)

    def delete_boxes_on_image(self, image_name: str):
        name = image_name.split(".")[0]
        if name in self.bbox_data:
            del self.bbox_data[name]
        self._write_bbox_data_to_file()

def rectangles_are_similar(rect1: QtCore.QRect, rect2: QtCore.QRect, tolerance = 1) -> bool:
    left1, top1, right1, bottom1 = rect1.left(), rect1.top(), rect1.right(), rect1.bottom()
    left2, top2, right2, bottom2 = rect2.left(), rect2.top(), rect2.right(), rect2.bottom()
    return (
        abs(left1 - left2) <= tolerance and abs(top1 - top2) <= tolerance and
        abs(right1 - right2) <= tolerance and abs(bottom1 - bottom2) <= tolerance
    )


if __name__ == "__main__":
    bbox_list = BBoxList()

    rect1 = QtCore.QRect(10, 20, 30, 40)
    rect2 = QtCore.QRect(50, 60, 70, 80)
    rect3 = QtCore.QRect(15, 25, 35, 45)

    bbox_list.add_bbox(rect1, "cat", "images/img1.jpg")
    bbox_list.add_bbox(rect2, "dog", "images/img1.jpg")
    bbox_list.add_bbox(rect3, "cat", "images/img2.jpg")