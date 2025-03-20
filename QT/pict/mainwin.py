from PyQt6 import QtWidgets, QtGui
from viewer import ImageModelViewer

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Image Viewer")
        self.setGeometry(100, 100, 800, 600)

        self.setWindowIcon(QtGui.QIcon(":/main"))
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.image_model_viewer = ImageModelViewer(self)