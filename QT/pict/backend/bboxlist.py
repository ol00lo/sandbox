from typing import Dict, List, Tuple
from pathlib import Path
from PyQt6 import QtCore
import os
from backend.box import Box

class BBoxList:
    def __init__(self, folder: str = "", output_file: str = "") -> None:
        self.first_line = "Path,Label,x1,y1,x2,y2\n"
        if not output_file == "":
            abs_folder = os.path.abspath(folder)
            self.output_file = os.path.join(abs_folder, output_file)
            self.bbox_data: Dict[str, List[Box]] = {}
            self._check_out_file()

        else:
            csv_files = [f for f in os.listdir(os.path.abspath(folder)) if f.lower().endswith('.csv')]
            if csv_files:
                self.output_file = os.path.join(abs_folder, csv_files[0])
                self._check_out_file()

    def add_bbox(self, bbox: Box, name):
        if name not in self.bbox_data:
            self.bbox_data[name] = []
        self.bbox_data[name].append(bbox)

    def new_bbox(self, bbox: Box, name, ):
        self.add_bbox(bbox, name)
        with open(self.output_file, 'a') as f:
            line = self.get_line(name, bbox)
            f.write(line)

        print(f"BBox appended to: {Path(self.output_file).absolute()}")

    def delete_bbox(self, box: Box, name):
        if name not in self.bbox_data:
            return False

        for c in self.bbox_data[name]:
            if box == c:
                self.bbox_data[name].remove(c)
                self._write_bbox_data_to_file()
                print(f"BBox deleted from: {Path(self.output_file).absolute()}")
                return True

        return False

    def _read_bbox_data_from_file(self, lines):
        for line in lines:
            line = line.strip()
            if line == self.first_line:
                continue
            name, label, x1, y1, x2, y2 = line.split(',')
            bbox = Box(label, QtCore.QRectF(float(x1), float(y1), float(x2) - float(x1), float(y2)-float(y1)))
            self.add_bbox(bbox, name)

    def _write_bbox_data_to_file(self):
        with open(self.output_file, 'w') as f:
            f.write(self.first_line)
            for bbox_name, bboxes in self.bbox_data.items():
                for bbox in bboxes:
                    line = self.get_line(bbox_name, bbox)
                    f.write(line)

    def get_line(self, name, bbox):
        return f'{name},{bbox.label}, {bbox.left()}, {bbox.top()}, {bbox.right()}, {bbox.bottom()}\n'

    def _check_out_file(self):
        if  os.path.exists(self.output_file):
            with open(self.output_file, 'r') as f:
                line = f.readline()
                if line ==  self.first_line:
                    self._read_bbox_data_from_file(f.readlines())
                    return
        with open(self.output_file, 'w') as f:
            f.write(self.first_line)

    def update_bbox(self, old_box: Box, new_box: Box, name):
        if name not in self.bbox_data:
            return False

        for c in self.bbox_data[name]:
            if old_box == c:
                c.setCoords(new_box.left(), new_box.top(), new_box.right(), new_box.bottom())
                self._write_bbox_data_to_file()
                print(f"BBox updated in: {Path(self.output_file).absolute()}")
                return True

        return False

    def delete_boxes_on_image(self, name: str):
        if name in self.bbox_data:
            del self.bbox_data[name]
        self._write_bbox_data_to_file()

if __name__ == "__main__":
    bbox_list = BBoxList()

    rect1 = QtCore.QRect(10, 20, 30, 40)
    rect2 = QtCore.QRect(50, 60, 70, 80)
    rect3 = QtCore.QRect(15, 25, 35, 45)

    bbox_list.new_bbox(rect1, "cat", "images/img1.jpg")
    bbox_list.new_bbox(rect2, "dog", "images/img1.jpg")
    bbox_list.new_bbox(rect3, "cat", "images/img2.jpg")