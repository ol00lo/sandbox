from typing import Dict, List, Tuple
from pathlib import Path
from PyQt6 import QtCore, QtWidgets, QtGui
import os

class Box(QtWidgets.QGraphicsRectItem):
    def __init__(self, rect=None, label="", image_path="", parent=None):
        rect = rect if rect else QtCore.QRectF()
        super().__init__(rect, parent)
        self.label = label
        self.name = image_path.split("\\")[-1].split(".")[0]

        self.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.yellow, 2, QtCore.Qt.PenStyle.SolidLine))
        self.setFlag(QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)

    def update_label(self, new_label):
        self.label = new_label

class BBoxList:
    def __init__(self, folder: str = "", output_file: str = "") -> None:
        if not output_file == "":
            abs_folder = os.path.abspath(folder)
            self.output_file = os.path.join(abs_folder, output_file)
            self.bbox_data: Dict[str, List[Tuple[str, QtCore.QRect]]] = {}
            with open(self.output_file, 'w') as f:
                f.write("Path,n_boxes,Label,x1,y1,x2,y2\n")

    def add_bbox(self, bbox: QtCore.QRect, label: str, image_name: str):
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


if __name__ == "__main__":
    bbox_list = BBoxList()

    rect1 = QtCore.QRect(10, 20, 30, 40)
    rect2 = QtCore.QRect(50, 60, 70, 80)
    rect3 = QtCore.QRect(15, 25, 35, 45)

    bbox_list.add_bbox(rect1, "cat", "images/img1.jpg")
    bbox_list.add_bbox(rect2, "dog", "images/img1.jpg")
    bbox_list.add_bbox(rect3, "cat", "images/img2.jpg")