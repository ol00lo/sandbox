from PyQt6 import QtCore, QtWidgets, QtGui
from actions import LoadImagesAction, DeleteAllImagesAction, RenameFileAction, DeleteImageAction
from state import State

class TableViewer (QtWidgets.QWidget):
    def __init__(self, parent):
        super().__init__(parent)

        self.table_view = QtWidgets.QTableView()
        self.table_view.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.DoubleClicked)
        self.table_view.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.table_view.setContextMenuPolicy(QtCore.Qt.ContextMenuPolicy.CustomContextMenu)
        self.table_view.customContextMenuRequested.connect(self.show_context_menu)
        self.table_view.setMouseTracking(True)
        self.table_view.viewport().installEventFilter(self)

        self.load_images_button = QtWidgets.QPushButton("Open Folder")
        self.load_images_button.clicked.connect(State().actions["LoadImages"].do)

        self.delete_images_button = QtWidgets.QPushButton("Delete All Images")
        self.delete_images_button.clicked.connect(State().actions["DeleteAllImages"].do)

        main_layout = QtWidgets.QVBoxLayout()
        main_layout.addWidget(State().filter_line_edit)
        main_layout.addWidget(self.table_view)
        main_layout.addWidget(self.load_images_button)
        main_layout.addWidget(self.delete_images_button)

        main_layout.setContentsMargins(10, 0, 10, 0)
        table_widget = QtWidgets.QWidget()
        table_widget.setLayout(main_layout)
        self.setLayout(main_layout)

    def show_context_menu(self, position):
            selected_rows = self.table_view.selectionModel().selectedRows()
            cur_row = selected_rows[0]
            context_menu = QtWidgets.QMenu(self.table_view)

            rename_action = State().actions["RenameFile"]
            delete_action = State().actions["DeleteImage"]

            context_menu.addAction(delete_action)
            delete_action.triggered.connect(lambda: delete_action.do(selected_rows))

            context_menu.addAction(rename_action)
            rename_action.triggered.connect(lambda: rename_action.do(cur_row))

            context_menu.exec(self.table_view.viewport().mapToGlobal(position))

            rename_action.triggered.disconnect()
            delete_action.triggered.disconnect()

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key.Key_Delete:
            selected_rows = self.table_view.selectionModel().selectedRows()
            State().actions["DeleteImage"].do(selected_rows)
        else:
            super().keyPressEvent(event)

    def init_connections(self):
        self.table_view.setModel(State().proxy_model)
        self.table_view.selectionModel().selectionChanged.connect(State().actions["LoadImages"].on_selection_changed)
        self.table_view.setSortingEnabled(True)
        for column_index in range(len(State().model.columns)):
            self.table_view.horizontalHeader().setSectionResizeMode(column_index, QtWidgets.QHeaderView.ResizeMode.Stretch)
