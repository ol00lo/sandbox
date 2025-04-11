from PyQt6 import QtWidgets, QtGui, QtCore
from imageviewer import ImageViewer
from tableviewer import TableViewer
from state import State

class MainWindow(QtWidgets.QMainWindow):
    image_selected = QtCore.pyqtSignal(str)
    curr_dir_signal = QtCore.pyqtSignal(str)
    load_images_signal = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Image Viewer")
        self.setGeometry(100, 100, 800, 600)

        self.setWindowIcon(QtGui.QIcon(":/main"))
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)

        self.splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal, self.central_widget)
        State().init_actions(self)
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

        self.curr_dir_signal.connect(self.show_folder_name)
        self.image_selected.connect(self.image_model_viewer.display_image)
        self.image_model_viewer.coordinates_clicked.connect(self.show_coordinates)
        self.load_images_signal.connect(self.table_viewer.init_connections)

        self.create_toolbar()

    def create_toolbar(self):
        toolbar = self.addToolBar("Main Toolbar")

        toolbar.addAction(State().actions["LoadImages"])
        State().actions["LoadImages"].triggered.connect(State().actions["LoadImages"].do)

        toolbar.addAction(State().actions["DeleteAllImages"])
        State().actions["DeleteAllImages"].triggered.connect(State().actions["DeleteAllImages"].do)

        toolbar.addAction(State().actions["AddImage"])
        State().actions["AddImage"].triggered.connect(State().actions["AddImage"].do)

        toolbar.addSeparator()

    def show_coordinates(self, x, y):
        self.status_bar.showMessage(f"X: {x}, Y: {y}")

    def show_folder_name(self, dir_path):
        if dir_path:
            self.setWindowTitle(f"Image Viewer - {dir_path}")
        else:
            self.setWindowTitle("Image Viewer")
