from PyQt6 import QtWidgets, QtGui, QtCore
from backend.state import State
from box import Box
import os

class ImageModel(QtWidgets.QGraphicsScene):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.current_image_path = ""
        State().signals.delete_box_signal.connect(self.display_image)

    def display_image(self, image_path):
        self.clear()
        self.current_image_path = image_path
        pixmap = QtGui.QPixmap(image_path)

        pixmap_item = self.addPixmap(pixmap)
        self.setSceneRect(pixmap_item.boundingRect())
        self.draw_boxes()

    def draw_boxes(self):
        boxes = self.find_rects(os.path.dirname(self.current_image_path))
        for x1, y1, x2, y2, label in boxes:
            rect = QtCore.QRectF(x1, y1, x2 - x1, y2 - y1)
            self.addItem(Box(rect=rect, label=label, image_path=self.current_image_path))

    def find_rects(self, directory):
        if directory == '': return []
        csv_file_path = next((os.path.join(directory, f) for f in os.listdir(directory) if f.endswith(".csv")), None)
        if not csv_file_path:
            return []

        rects = []
        with open(csv_file_path, "r", encoding="utf-8") as f:
            for line in f.readlines()[1:]:
                parts = line.strip().split(",")
                if len(parts) < 7:
                    continue
                image_name = parts[0][1:-1]
                if image_name != os.path.splitext(os.path.basename(self.current_image_path))[0]:
                    continue
                try:
                    label = parts[2][1:-1]
                    coords = list(map(int, parts[3:7]))
                    rects.append((*coords, label))
                except ValueError:
                    continue
        return rects
