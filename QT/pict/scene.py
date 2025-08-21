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

        State().signals.add_label_signal.connect(self.create_label_item)

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

            if box_item.scene() is None: self.addItem(box_item)
            if State().need_labels:
                self.add_labels(box_item, box.label)

    def add_labels(self, box, label):
        text_item = self.create_label_item(label, box)
        if text_item.scene() is None: self.addItem(text_item)

    def create_label_item(self, label_text, rect, is_hover=False, callback=None):
        text_item = QtWidgets.QGraphicsSimpleTextItem(label_text, rect)
        text_item.setFont(DrawState().label_font)
        text_item.setBrush(QtGui.QBrush(DrawState().label_color))
        text_item.setPen(DrawState().label_pen)
        text_item.setFlag(QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)

        scene = self.scene() if hasattr(self, 'scene') else self
        view = scene.views()[0] if scene and scene.views() else None
        scale_x = view.transform().m11() if view else 1.0
        scale_y = view.transform().m22() if view else 1.0

        padding_pixels = 2.0
        padding_scene_x = padding_pixels / scale_x
        padding_scene_y = padding_pixels / scale_y

        text_height_pixels = text_item.boundingRect().height()
        text_height_scene = text_height_pixels / scale_y
        x_pos = rect.rect().left() + padding_scene_x
        y_pos = rect.rect().top() - text_height_scene - padding_scene_y
        text_item.setPos(QtCore.QPointF(x_pos, y_pos))

        bg_color = DrawState().hover_label_background_color if is_hover else DrawState().label_background_color

        background_item = QtWidgets.QGraphicsRectItem(text_item)
        background_item.setBrush(QtGui.QBrush(bg_color))
        background_item.setPen(QtGui.QPen(QtCore.Qt.PenStyle.NoPen))
        background_item.setFlag(QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        background_item.setFlag(QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemStacksBehindParent)

        text_rect = text_item.boundingRect()
        background_item.setRect(
            text_rect.adjusted(-padding_pixels, -padding_pixels, padding_pixels, padding_pixels)
        )

        if callback:
            callback(text_item)

        return text_item

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