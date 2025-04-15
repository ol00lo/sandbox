from PyQt6 import QtCore, QtWidgets, QtGui
from scene import ImageModel

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
        else:
            self.setCursor(self.default_cursor)
        super().mouseMoveEvent(event)

    def leaveEvent(self, event):
        self.setCursor(self.default_cursor)
        super().leaveEvent(event)