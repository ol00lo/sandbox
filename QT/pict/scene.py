from PyQt6 import QtWidgets, QtGui, QtCore
from backend.state import State
from boxgritem import BoxGraphicsItem
import os
from backend.box import Box

class ImageModel(QtWidgets.QGraphicsScene):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.current_image_path = ""
        State().signals.change_boxes_signal.connect(self.update_boxes)
        State().signals.delete_box_signal.connect(self.update_boxes)
        State().signals.create_box_signal.connect(self.update_boxes)

    def display_image(self):
        self.clear()
        self.current_image_path = State().get_path()
        pixmap = QtGui.QPixmap(self.current_image_path)

        pixmap_item = self.addPixmap(pixmap)
        self.setSceneRect(pixmap_item.boundingRect())
        self.draw_boxes()

    def update_boxes(self):
        for item in self.items():
            if isinstance(item, BoxGraphicsItem):
                self.removeItem(item)
            elif isinstance(item, QtWidgets.QGraphicsSimpleTextItem):
                if item.parentItem() and isinstance(item.parentItem(), BoxGraphicsItem):
                    self.removeItem(item)
        self.draw_boxes()

    def draw_boxes(self):
        boxes = self.find_rects(os.path.dirname(self.current_image_path))
        for box in boxes:
            box_item = BoxGraphicsItem(box=box, image_path=self.current_image_path)
            self.addItem(box_item)
            if State().need_labels:
                self.add_labels(box_item, box.label)

    def add_labels(self, box, label):
        width = self.sceneRect().width()
        font_size = max(8, width * 0.05)

        font = QtGui.QFont("Times New Roman")
        font.setPointSizeF(font_size)

        text_item = QtWidgets.QGraphicsSimpleTextItem(label, box)

        step = width * 0.01
        local_pos = QtCore.QPointF(box.rect().right() + step, box.rect().top() - step)
        text_item.setPos(local_pos)

        pen = QtGui.QPen(QtCore.Qt.GlobalColor.black, 1, QtCore.Qt.PenStyle.SolidLine)
        text_item.setPen(pen)
        text_item.setFont(font)
        self.addItem(text_item)

    def find_rects(self, directory):
        if directory == '': return []
        csv_file_path = next((os.path.join(directory, f) for f in os.listdir(directory) if f.endswith(".csv")), None)
        if not csv_file_path:
            return []

        rects = []
        with open(csv_file_path, "r", encoding="utf-8") as f:
            for line in f.readlines()[1:]:
                parts = line.strip().split(",")
                if len(parts) < 6:
                    continue
                image_name = parts[0]
                if image_name != os.path.basename(self.current_image_path):
                    continue
                try:
                    label = parts[1]
                    coords = list(map(float, parts[2:6]))
                    rect = QtCore.QRectF(coords[0], coords[1], coords[2] - coords[0], coords[3] - coords[1])
                    rects.append(Box( label, rect))
                except ValueError:
                    continue
        return rects
