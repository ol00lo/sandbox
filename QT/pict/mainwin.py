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

        self.table_viewer.curr_dir_signal.connect(self.show_folder_name)
        self.table_viewer.image_selected.connect(self.image_model_viewer.display_image)
        self.image_model_viewer.coordinates_clicked.connect(self.show_coordinates)

        self.create_toolbar()

    def create_toolbar(self):
        self.load_action = LoadImagesAction(self.table_viewer)
        self.load_action.triggered.connect(self.load_action.do)

        self.delete_action = DeleteAllImagesAction(self.table_viewer)
        self.delete_action.triggered.connect(self.delete_action.do)

        self.add_action = AddImageAction(self)
        self.add_action.triggered.connect(self.add_action.do)

        toolbar = self.addToolBar("Main Toolbar")

        toolbar.addAction(self.load_action)
        toolbar.addAction(self.delete_action)
        toolbar.addAction(self.add_action)

        toolbar.addSeparator()

    def show_coordinates(self, x, y):
        self.status_bar.showMessage(f"X: {x}, Y: {y}")

    def show_folder_name(self, dir_path):
        if dir_path:
            self.setWindowTitle(f"Image Viewer - {dir_path}")
        else:
            self.setWindowTitle("Image Viewer")