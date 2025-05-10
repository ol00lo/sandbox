from PyQt6 import QtCore, QtWidgets, QtGui
from scene import ImageModel
from backend.bbox import BBoxList

class ImageViewer(QtWidgets.QGraphicsView):
    coordinates_clicked = QtCore.pyqtSignal(int, int)
    box_created = QtCore.pyqtSignal(QtCore.QRect, str, str)
    box_deleted = QtCore.pyqtSignal(str)

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
        t = self.image_model.sceneRect().width() // 90
        self.box_pen = QtGui.QPen(QtCore.Qt.GlobalColor.yellow, t, QtCore.Qt.PenStyle.SolidLine)
        self.box_saver = BBoxList()
        self.box_created.connect(self.box_saver.add_bbox)

    def display_image(self, image_path):
        self.image_model.display_image(image_path)
        self.fitInView(self.image_model.sceneRect(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.image_model.items():
            self.fitInView(self.image_model.sceneRect(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)

    def mouseMoveEvent(self, event):
        item = self.itemAt(event.pos())

        if isinstance(item, QtWidgets.QGraphicsPixmapItem):
            self.setCursor(self.cross_cursor)

            scene_pos = self.mapToScene(event.pos())

            img_pos = item.mapFromScene(scene_pos)
            pixmap = item.pixmap()

            x = max(0, min(int(img_pos.x()), pixmap.width() - 1))
            y = max(0, min(int(img_pos.y()), pixmap.height() - 1))

            self.coordinates_clicked.emit(x, y)

            if self.drawing and self.current_box:
                end_point = self.mapToScene(event.pos())
                rect = QtCore.QRectF(self.start_point, end_point).normalized()
                self.current_box.setRect(rect)
        else:
            self.setCursor(self.default_cursor)
        super().mouseMoveEvent(event)

    def leaveEvent(self, event):
        self.setCursor(self.default_cursor)
        super().leaveEvent(event)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self.drawing = True
            self.start_point = self.mapToScene(event.pos())
            self.current_box = QtWidgets.QGraphicsRectItem()
            self.current_box.setPen(self.box_pen)
            self.image_model.addItem(self.current_box)
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton and self.drawing:
            self.drawing = False
            if self.current_box:
                label, ok = QtWidgets.QInputDialog.getText(self, "Label", "Enter label:")
                if not ok:
                    self.image_model.removeItem(self.current_box)
                    self.current_box = None
                    return

                rect = self.current_box.rect().toRect()
                path = self.image_model.current_image_path
                self.box_created.emit(rect, label, path)

        self.image_model.removeItem(self.current_box)
        self.current_box = None
        super().mouseReleaseEvent(event)
