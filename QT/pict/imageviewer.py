from PyQt6 import QtCore, QtWidgets
from scene import ImageModel

class ImageViewer(QtWidgets.QWidget):
    coordinates_clicked = QtCore.pyqtSignal(str) 

    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent
        self.image_viewer = ImageModel(self)

        self.coordinates_clicked.emit(f"X: {0}, Y: {0}")
        
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.image_viewer)     

    def display_image(self, image_path):
        self.image_viewer.display_image(image_path)

    def update_coordinates(self, x, y):
        self.coordinates_clicked.emit(f"X: {x}, Y: {y}")