from PyQt6 import QtWidgets, QtGui, QtCore
from PIL import Image

class ImageScene(QtWidgets.QGraphicsScene):
    def __init__(self, parent=None):
        super().__init__(parent)

    def display_image(self, image_path):
        self.clear()
        self.current_image_path = image_path
        pixmap = QtGui.QPixmap(image_path)

        pixmap_item = self.addPixmap(pixmap)
        self.setSceneRect(pixmap_item.boundingRect())

    def cur_size(self):
        try:
            with Image.open(self.current_image_path) as image:
                width, height = image.size
                return width, height
        except Exception as e:
            return 0, 0

class ImageModel(QtWidgets.QGraphicsView):
    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent
        self.image_scene = ImageScene(self)
        self.setScene(self.image_scene)

    def display_image(self, image_path):
        self.image_scene.display_image(image_path)
        self.setMouseTracking(True)

        self.original_pixmap = self.image_scene.cur_size()

        self.fitInView(self.image_scene.sceneRect(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.image_scene.items():
            self.fitInView(self.image_scene.sceneRect(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)

    def mouseMoveEvent(self, event):
        scene_pos = self.mapToScene(event.pos())

        current_image_item = self.image_scene.items()
        if current_image_item:
            pixmap_item = current_image_item[0]

            original_width = self.original_pixmap[0]
            original_height = self.original_pixmap[1]

            cursor_x = int(scene_pos.x() - pixmap_item.x())
            cursor_y = int(scene_pos.y() - pixmap_item.y())

            cursor_x = max(0, min(cursor_x, original_width - 1))
            cursor_y = max(0, min(cursor_y, original_height - 1))

            self.parent.update_coordinates(cursor_x, cursor_y)

        super().mouseMoveEvent(event)
