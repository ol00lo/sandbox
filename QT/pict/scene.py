from PyQt6 import QtWidgets, QtGui, QtCore

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

class ImageViewer(QtWidgets.QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.image_scene = ImageScene(self)
        self.setScene(self.image_scene)

        self.coord_label = QtWidgets.QLabel(self)

    def display_image(self, image_path):
        self.image_scene.display_image(image_path)
        # if image_path is not None: self.setMouseTracking(True)
        # else: self.setMouseTracking(False)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        current_image_path = self.image_scene.items()
        if current_image_path:
            image_path = current_image_path[0].pixmap().toImage()
            self.display_image(image_path)
    
    def mouseMoveEvent(self, event):
        scene_pos = self.mapToScene(event.pos())
        self.coord_label.setText(f'X: {scene_pos.x():.2f}, Y: {scene_pos.y():.2f}')
        super().mouseMoveEvent(event)