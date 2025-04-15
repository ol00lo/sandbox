from PyQt6 import QtWidgets, QtGui

class ImageModel(QtWidgets.QGraphicsScene):
    def __init__(self, parent=None):
        super().__init__(parent)

    def display_image(self, image_path):
        self.clear()
        self.current_image_path = image_path
        pixmap = QtGui.QPixmap(image_path)

        pixmap_item = self.addPixmap(pixmap)
        self.setSceneRect(pixmap_item.boundingRect())

