from PyQt6 import QtWidgets, QtGui, QtCore
from PIL import Image

class ImageScene(QtWidgets.QGraphicsScene):
    def __init__(self, parent=None):
        super().__init__(parent)

    def display_image(self, image_path):
        self.clear()
        self.current_image_path = image_path
        pixmap = QtGui.QPixmap(image_path)
        view_width = self.parent().width()
        view_height = self.parent().height()

        scaled_pixmap = pixmap.scaled(view_width, view_height, QtCore.Qt.AspectRatioMode.KeepAspectRatio, QtCore.Qt.TransformationMode.SmoothTransformation)
        pixmap_item = self.addPixmap(scaled_pixmap)
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

    def resizeEvent(self, event):
        super().resizeEvent(event)
        current_image_items = self.image_scene.items()
        if current_image_items:
            self.image_scene.display_image(self.image_scene.current_image_path)

    def mouseMoveEvent(self, event):
        scene_pos = self.mapToScene(event.pos())

        current_image_item = self.image_scene.items()
        if current_image_item:
            pixmap_item = current_image_item[0]

            original_width = self.original_pixmap[0]
            original_height = self.original_pixmap[1]

            scaled_width = pixmap_item.boundingRect().width()
            scaled_height = pixmap_item.boundingRect().height()

            if scaled_width == 0 or scaled_height == 0:
                return
            x_scale = original_width / scaled_width
            y_scale = original_height / scaled_height

            cursor_x = int((scene_pos.x() - pixmap_item.x()) * x_scale)
            cursor_y = int((scene_pos.y() - pixmap_item.y()) * y_scale)

            cursor_x = max(0, min(cursor_x, original_width - 1))
            cursor_y = max(0, min(cursor_y, original_height - 1))

            self.parent.update_coordinates(cursor_x, cursor_y)

        super().mouseMoveEvent(event)
