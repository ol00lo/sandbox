from typing import Dict, List
from pathlib import Path
from PyQt6 import QtCore
import os
import csv
from backend.box import Box

class BBoxList:
    def __init__(self, folder: str = "", output_file: str = "") -> None:
        self.header = ["Path", "Label", "x1", "y1", "x2", "y2"]
        abs_folder = os.path.abspath(folder)

        if not output_file == "":
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

    def new_bbox(self, bbox: Box, name):
        self.add_bbox(bbox, name)

        with open(self.output_file, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            bbox.write(writer, name)

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

    def _read_bbox_data_from_file(self):
        with open(self.output_file, 'r', newline='') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)
            for row in reader:
                bbox = Box()
                name = bbox.read_and_ret_name(row)
                self.add_bbox(bbox, name)

    def _write_bbox_data_to_file(self):
        with open(self.output_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.header)
            for bbox_name, bboxes in self.bbox_data.items():
                for bbox in bboxes:
                    bbox.write(writer, bbox_name)

    def _check_out_file(self):
        if os.path.exists(self.output_file):
            with open(self.output_file, 'r', newline='') as csvfile:
                reader = csv.reader(csvfile)
                try:
                    header = next(reader)
                    if header == self.header:
                        self._read_bbox_data_from_file()
                        return
                except StopIteration:
                    pass

        with open(self.output_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.header)

    def update_bbox(self, old_box: Box, new_box: Box, name):
        if name not in self.bbox_data:
            return False

        for c in self.bbox_data[name]:
            if c == old_box:
                c.copy_coord(new_box)
                self._write_bbox_data_to_file()
                print(f"BBox updated in: {Path(self.output_file).absolute()}")
                return True

        return False

    def delete_boxes_on_image(self, name: str):
        if name in self.bbox_data:
            del self.bbox_data[name]
        self._write_bbox_data_to_file()

    def box_count(self, name: str):
        if name in self.bbox_data:
            return len(self.bbox_data[name])
        return 0