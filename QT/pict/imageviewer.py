from PyQt6 import QtCore, QtWidgets, QtGui
from scene import ImageModel
from backend.bbox import Box
from backend.state import State

class ImageViewer(QtWidgets.QGraphicsView):
    coordinates_clicked = QtCore.pyqtSignal(int, int)

    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent

        self.image_model = ImageModel(self)
        self.setScene(self.image_model)
        self.setMouseTracking(True)

        self.cross_cursor = QtGui.QCursor(QtCore.Qt.CursorShape.CrossCursor)
        self.default_cursor = QtGui.QCursor()
        self.setCursor(self.default_cursor)

        self.drawing = False
        self.start_point = QtCore.QPoint()
        self.current_box = None

    def display_image(self, image_path):
        self.image_model.display_image(image_path)
        self.fitInView(self.image_model.sceneRect(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.image_model.items():
            self.fitInView(self.image_model.sceneRect(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self.start_drawing(event)
        elif event.button() == QtCore.Qt.MouseButton.RightButton:
            item = self.itemAt(event.pos())
            if isinstance(item, Box):
                State().actions["DeleteBox"].do(item)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        item = self.itemAt(event.pos())
        if isinstance(item, QtWidgets.QGraphicsPixmapItem):
            self.setCursor(self.cross_cursor)
            self.track_coordinates(event)
        if self.drawing and self.current_box:
            self.update_current_box(event)
        if not isinstance(item, QtWidgets.QGraphicsPixmapItem):
            self.setCursor(self.default_cursor)
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton and self.drawing:
            self.finish_drawing()

        super().mouseReleaseEvent(event)

    def track_coordinates(self, event):
        pos = event.pos()
        scene_pos = self.mapToScene(pos)
        item = self.itemAt(pos)
        img_pos = item.mapFromScene(scene_pos)
        pixmap = item.pixmap()

        x = max(0, min(int(img_pos.x()), pixmap.width() - 1))
        y = max(0, min(int(img_pos.y()), pixmap.height() - 1))

        self.coordinates_clicked.emit(x, y)

    def update_current_box(self, event):
        end_point = self.mapToScene(event.pos())
        scene_rect = self.image_model.sceneRect()

        x = max(scene_rect.left(), min(end_point.x(), scene_rect.right()))
        y = max(scene_rect.top(), min(end_point.y(), scene_rect.bottom()))
        clamped_end_point = QtCore.QPointF(x, y)

        rect = QtCore.QRectF(self.start_point, clamped_end_point).normalized()
        self.current_box.setRect(rect)

    def start_drawing(self, event):
        self.drawing = True
        scene_rect = self.image_model.sceneRect()
        start_point = self.mapToScene(event.pos())

        x = max(scene_rect.left(), min(start_point.x(), scene_rect.right()))
        y = max(scene_rect.top(), min(start_point.y(), scene_rect.bottom()))
        self.start_point = QtCore.QPointF(x, y)

        self.current_box = Box(image_path=self.image_model.current_image_path)
        self.image_model.addItem(self.current_box)

    def finish_drawing(self):
        self.drawing = False

        if not self.current_box:
            return

        label, ok = QtWidgets.QInputDialog.getText(self, "Label", "Enter label:")
        if not ok or label == "":
            self.cancel_drawing()
            return
        self.current_box.update_label(label)
        State().actions["CreateBox"].do(self.current_box)
        self.current_box = None

    def cancel_drawing(self):
        self.image_model.removeItem(self.current_box)
        self.current_box = None

    def leaveEvent(self, event):
        self.setCursor(self.default_cursor)
        super().leaveEvent(event)
