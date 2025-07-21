from PyQt6 import QtWidgets, QtGui, QtCore
from imageviewer import ImageViewer
from tableviewer import TableViewer
from backend.state import State
import resources

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

        self.table_viewer.image_selected.connect(self.image_model_viewer.image_model.set_selected_image)

        self.splitter.addWidget(self.image_model_viewer)
        self.splitter.addWidget(self.table_viewer)

        self.splitter.setSizes([500, 300])
        self.splitter.setStyleSheet("QSplitter::handle { background: lightgray; }")

        layout = QtWidgets.QVBoxLayout(self.central_widget)
        layout.addWidget(self.splitter)

        self.status_bar = QtWidgets.QStatusBar(self)
        self.setStatusBar(self.status_bar)

        self.image_model_viewer.coordinates_clicked.connect(self.show_coordinates)

        self.table_viewer.image_selected.connect(self.image_model_viewer.display_image)
        self.create_toolbar()

    def create_toolbar(self):
        #toolbar = self.addToolBar("Main Toolbar")
        self.toolbar = self.addToolBar("Main Toolbar")
        toolbar = self.toolbar
        toolbar.setToolButtonStyle(QtCore.Qt.ToolButtonStyle.ToolButtonIconOnly)
        #toolbar.setToolButtonStyle(QtCore.Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        toolbar.setIconSize(QtCore.QSize(15, 15))

        undo_action = QtGui.QAction(QtGui.QIcon(":/undo"), "Undo", self)
        undo_action.setShortcut(QtGui.QKeySequence.StandardKey.Undo)
        undo_action.triggered.connect(State().undo_redo_manager.undo)
        toolbar.addAction(undo_action)

        redo_action = QtGui.QAction(QtGui.QIcon(":/redo"), "Redo", self)
        redo_action.setShortcut(QtGui.QKeySequence.StandardKey.Redo)
        redo_action.triggered.connect(State().undo_redo_manager.redo)
        toolbar.addAction(redo_action)

        toolbar.addSeparator()

        load_action = State().actions["LoadImages"]
        toolbar.addAction(load_action)
        load_action.triggered.connect(lambda: State().do_action("LoadImages"))

        deleteall_action = State().actions["DeleteAllImages"]
        toolbar.addAction(deleteall_action)
        deleteall_action.triggered.connect(lambda: State().do_action("DeleteAllImages"))

        add_action = State().actions["AddImage"]
        toolbar.addAction(add_action)
        add_action.triggered.connect(lambda: State().do_action("AddImage"))

        toolbar.addSeparator()

        self.need_labels_checkbox = QtWidgets.QCheckBox("Labels")
        self.need_labels_checkbox.setChecked(False)
        self.need_labels_checkbox.stateChanged.connect(self.on_need_labels_changed)

        checkbox_action = QtWidgets.QWidgetAction(self)
        checkbox_action.setDefaultWidget(self.need_labels_checkbox)

        toolbar.addAction(checkbox_action)

    def on_need_labels_changed(self, state):
        State().need_labels = (state == QtCore.Qt.CheckState.Checked.value)
        self.image_model_viewer.update_image()


    def show_coordinates(self, x, y):
        self.status_bar.showMessage(f"X: {x}, Y: {y}")

    def show_folder_name(self, dir_path):
        if dir_path:
            self.setWindowTitle(f"Image Viewer - {dir_path}")
        else:
            self.setWindowTitle("Image Viewer")

    def open_folder(self, dir_path = "tests/test_images"):
        State().do_action("LoadImages", dir_path)
