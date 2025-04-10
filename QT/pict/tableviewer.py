from PyQt6 import QtCore, QtWidgets, QtGui
from actions import LoadImagesAction, DeleteAllImagesAction, RenameFileAction, DeleteImageAction
from state import State

class TableViewer (QtWidgets.QWidget):
    image_selected = QtCore.pyqtSignal(str)
    curr_dir_signal = QtCore.pyqtSignal(str)

    def __init__(self, parent):
        super().__init__(parent)

        self.table_view = QtWidgets.QTableView()
        self.table_view.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.DoubleClicked)
        self.table_view.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.table_view.setContextMenuPolicy(QtCore.Qt.ContextMenuPolicy.CustomContextMenu)
        self.table_view.customContextMenuRequested.connect(self.show_context_menu)
        self.table_view.setMouseTracking(True)
        self.table_view.viewport().installEventFilter(self)

        self.load_images_button = QtWidgets.QPushButton("Load Images")
        self.load_images_button.clicked.connect(LoadImagesAction(self).do)

        self.delete_images_button = QtWidgets.QPushButton("Delete All Images")
        self.delete_images_button.clicked.connect(DeleteAllImagesAction(self).do)

        main_layout = QtWidgets.QVBoxLayout()
        main_layout.addWidget(State().filter_line_edit)
        main_layout.addWidget(self.table_view)
        main_layout.addWidget(self.load_images_button)
        main_layout.addWidget(self.delete_images_button)

        table_widget = QtWidgets.QWidget()
        table_widget.setMaximumWidth(400)
        table_widget.setLayout(main_layout)
        self.setLayout(main_layout)

        self.delete_action = DeleteImageAction(self)
        self.rename_action = RenameFileAction(self)

    def show_context_menu(self, position):
            selected_rows = self.table_view.selectionModel().selectedRows()
            cur_row = selected_rows[0]
            context_menu = QtWidgets.QMenu(self.table_view)

            context_menu.addAction(self.delete_action)
            self.delete_action.triggered.connect(lambda: self.delete_action.do(selected_rows))

            context_menu.addAction(self.rename_action)
            self.rename_action.triggered.connect(lambda: self.rename_action.do(cur_row))

            context_menu.exec(self.table_view.viewport().mapToGlobal(position))

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key.Key_Delete:
            index = self.table_view.currentIndex()
            if index.isValid():
                self.delete_action.trigger()
        else:
            super().keyPressEvent(event)