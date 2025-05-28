from typing import Dict, List, Tuple
from pathlib import Path
from PyQt6 import QtCore
import os
from box import Box

class BBoxList:
    def __init__(self, folder: str = "", output_file: str = "") -> None:
        if not output_file == "":
            abs_folder = os.path.abspath(folder)
            self.output_file = os.path.join(abs_folder, output_file)
            self.bbox_data: Dict[str, List[Tuple[str, QtCore.QRect]]] = {}
            with open(self.output_file, 'w') as f:
                f.write("Path,n_boxes,Label,x1,y1,x2,y2\n")

    def add_bbox(self, box: Box):
        image_name = box.name
        label = box.label
        bbox = box.rect().toRect()

        if image_name not in self.bbox_data:
            self.bbox_data[image_name] = []
        self.bbox_data[image_name].append((label, bbox))

        with open(self.output_file, 'a') as f:
            n_boxes = len(self.bbox_data[image_name])
            line = (
                f'"{image_name}",{n_boxes},"{label}",'
                f'{bbox.left()},{bbox.top()},'
                f'{bbox.right()},{bbox.bottom()}\n'
            )
            f.write(line)

        print(f"BBox appended to: {Path(self.output_file).absolute()}")

    def delete_bbox(self, box: Box):
        name = box.name
        label = box.label
        rect = box.rect().toRect()

        if name not in self.bbox_data:
            return False

        for bbox_label, bbox_rect in self.bbox_data[name]:
            if label == bbox_label and rectangles_are_similar(rect, bbox_rect):
                self.bbox_data[name].remove((bbox_label, bbox_rect))
                self._write_bbox_data_to_file()
                print(f"BBox deleted from: {Path(self.output_file).absolute()}")
                return True

        return False

    def _write_bbox_data_to_file(self):
        with open(self.output_file, 'w') as f:
            f.write("Path,n_boxes,Label,x1,y1,x2,y2\n")
            for bbox_name, bboxes in self.bbox_data.items():
                n_boxes = len(bboxes)
                for bbox_label, bbox_rect in bboxes:
                    line = (
                        f'"{bbox_name}",{n_boxes},"{bbox_label}",'
                        f'{bbox_rect.left()},{bbox_rect.top()},'
                        f'{bbox_rect.right()},{bbox_rect.bottom()}\n'
                    )
                    f.write(line)


def rectangles_are_similar(rect1: QtCore.QRect, rect2: QtCore.QRect, tolerance: int = 2) -> bool:
        expanded_rect1 = rect1.adjusted(-tolerance, -tolerance, tolerance, tolerance)
        expanded_rect2 = rect2.adjusted(-tolerance, -tolerance, tolerance, tolerance)
        return expanded_rect1.intersects(expanded_rect2)

if __name__ == "__main__":
    bbox_list = BBoxList()

    rect1 = QtCore.QRect(10, 20, 30, 40)
    rect2 = QtCore.QRect(50, 60, 70, 80)
    rect3 = QtCore.QRect(15, 25, 35, 45)

    bbox_list.add_bbox(rect1, "cat", "images/img1.jpg")
    bbox_list.add_bbox(rect2, "dog", "images/img1.jpg")
    bbox_list.add_bbox(rect3, "cat", "images/img2.jpg")