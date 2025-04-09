from PyQt6 import QtWidgets, QtGui, QtCore
from imageviewer import ImageViewer
from tableviewer import TableViewer
from actions import LoadImagesAction, DeleteAllImagesAction, AddImageAction

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Image Viewer")
        self.setGeometry(100, 100, 800, 600)

        self.setWindowIcon(QtGui.QIcon(":/main"))
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)

        self.splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal, self.central_widget)

        self.image_model_viewer = ImageViewer(self)
        self.table_viewer = TableViewer(self)

        self.splitter.addWidget(self.image_model_viewer)
        self.splitter.addWidget(self.table_viewer)

        self.splitter.setSizes([500, 300])
        self.splitter.setStyleSheet("QSplitter::handle { background: lightgray; }")

        layout = QtWidgets.QVBoxLayout(self.central_widget)
        layout.addWidget(self.splitter)

        self.status_bar = QtWidgets.QStatusBar(self)
        self.setStatusBar(self.status_bar)

        self.table_viewer.image_selected.connect(self.image_model_viewer.display_image)
        self.image_model_viewer.coordinates_clicked.connect(self.show_coordinates)
        self.create_toolbar()

    def create_toolbar(self):
        toolbar = self.addToolBar("Main Toolbar")
        
        toolbar.addAction(LoadImagesAction(self.table_viewer))
        toolbar.addAction(DeleteAllImagesAction(self.table_viewer))
        toolbar.addAction(AddImageAction(self.table_viewer.image_model, self))

        toolbar.addSeparator()

    def show_coordinates(self, x, y):
        self.status_bar.showMessage(f"X: {x}, Y: {y}")