from PyQt6 import QtWidgets, QtGui, QtCore
from backend.state import State
from boxgritem import BoxGraphicsItem
import os
from backend.box import Box
from backend.drawstate import DrawState

class ImageModel(QtWidgets.QGraphicsScene):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.current_image_path = ""
        State().signals.change_boxes_signal.connect(self.update_boxes)
        State().signals.delete_box_signal.connect(self.update_boxes)
        State().signals.create_box_signal.connect(self.update_boxes)

        State().signals.create_mask_signal.connect(self.start_drawing_mask)
        State().signals.delete_mask_signal.connect(self.remove_drawing_mask)
        State().signals.update_mask_signal.connect(self.update_drawing_mask)
        self.darken_mask = None

    def set_selected_image(self, name):
        self.current_image_path = State().get_path(name)

    def display_image(self):
        self.clear()
        pixmap = QtGui.QPixmap(self.current_image_path)

        pixmap_item = self.addPixmap(pixmap)
        self.setSceneRect(pixmap_item.boundingRect())
        self.draw_boxes()

    def update_boxes(self):
        for item in self.items():
            if isinstance(item, BoxGraphicsItem):
                self.removeItem(item)
            elif isinstance(item, QtWidgets.QGraphicsSimpleTextItem):
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
        text_item = QtWidgets.QGraphicsSimpleTextItem(label, box)
        text_item.setFont(DrawState().label_font)
        text_item.setBrush(QtGui.QBrush(DrawState().label_color))
        text_item.setPen(DrawState().label_pen)
        text_item.setFlag(QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)

        text_height = text_item.boundingRect().height()
        text_width = text_item.boundingRect().width()
        view_transform = self.views()[0].transform() if self.views() else None
        if view_transform:
            text_height /= view_transform.m22()
            text_width /= view_transform.m11()

        x_pos = box.rect().center().x() - text_width/2
        y_pos = box.rect().top() - text_height

        text_item.setPos(QtCore.QPointF(x_pos, y_pos))

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

    def start_drawing_mask(self, box_rect):
        if self.darken_mask:
            self.removeItem(self.darken_mask)

        mask = QtWidgets.QGraphicsPathItem()
        path = QtGui.QPainterPath()
        path.addRect(self.sceneRect())
        path.addRect(box_rect)
        mask.setPath(path)
        mask.setBrush(DrawState().dark_mask_color)
        mask.setPen(QtGui.QPen(QtCore.Qt.PenStyle.NoPen))

        self.darken_mask = mask
        self.addItem(mask)
        mask.setZValue(1)

    def update_drawing_mask(self, box_rect):
        if not self.darken_mask:
            return

        mask_path = QtGui.QPainterPath()
        mask_path.addRect(self.sceneRect())
        mask_path.addRect(box_rect)

        self.darken_mask.setPath(mask_path)

    def remove_drawing_mask(self):
        if self.darken_mask:
            self.removeItem(self.darken_mask)
            self.darken_mask = None