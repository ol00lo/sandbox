from PyQt6 import QtWidgets, QtGui, QtCore
from imageviewer import ImageViewer
from tableviewer import TableViewer
from backend.state import State
from boxsettings import BBoxSettingsDialog
from backend.drawstate import DrawState
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
        self.create_menus()

    def create_menus(self):
        menubar = self.menuBar()

        options_menu = menubar.addMenu("Options")

        bbox_settings_action = QtGui.QAction("BBox Settings...", self)
        bbox_settings_action.triggered.connect(self.get_boxes_parameters)
        options_menu.addAction(bbox_settings_action)

    def create_toolbar(self):
        self.toolbar = self.addToolBar("Main Toolbar")
        toolbar = self.toolbar
        toolbar.setToolButtonStyle(QtCore.Qt.ToolButtonStyle.ToolButtonIconOnly)
        #toolbar.setToolButtonStyle(QtCore.Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        toolbar.setIconSize(QtCore.QSize(15, 15))

        self.undo_action = QtGui.QAction(QtGui.QIcon(":/gundo"), "Undo", self)
        self.undo_action.setShortcut(QtGui.QKeySequence.StandardKey.Undo)
        self.undo_action.triggered.connect(State().undo_redo_manager.undo)
        toolbar.addAction(self.undo_action)

        self.redo_action = QtGui.QAction(QtGui.QIcon(":/gredo"), "Redo", self)
        self.redo_action.setShortcut(QtGui.QKeySequence.StandardKey.Redo)
        self.redo_action.triggered.connect(State().undo_redo_manager.redo)
        toolbar.addAction(self.redo_action)

        State().undo_redo_manager.undo_stack_empty_signal.connect(self.update_undo_icon)
        State().undo_redo_manager.redo_stack_empty_signal.connect(self.update_redo_icon)

        toolbar.addSeparator()

        toolbar.addAction(State().actions["LoadImages"])

        toolbar.addAction(State().actions["DeleteAllImages"])

        toolbar.addAction(State().actions["AddImage"])

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

    def update_undo_icon(self, is_empty):
        if is_empty:
            self.undo_action.setIcon(QtGui.QIcon(":/gundo"))
            self.undo_action.setEnabled(False)
        else:
            self.undo_action.setIcon(QtGui.QIcon(":/undo"))
            self.undo_action.setEnabled(True)

    def update_redo_icon(self, is_empty):
        if is_empty:
            self.redo_action.setIcon(QtGui.QIcon(":/gredo"))
            self.redo_action.setEnabled(False)
        else:
            self.redo_action.setIcon(QtGui.QIcon(":/redo"))
            self.redo_action.setEnabled(True)

    def get_boxes_parameters(self):
        dialog = BBoxSettingsDialog(self)
        if dialog.exec() == QtWidgets.QDialog.DialogCode.Accepted:
            res = dialog.get_settings()
            DrawState().line_color = res['line_color']
            DrawState().line_width = res['line_width']
            DrawState().line_style =  res['line_style']

            DrawState().label_size = res['label_size']
            DrawState().label_color = res['label_color']
            DrawState().label_type = res['label_type']

            self.image_model_viewer.update_image()