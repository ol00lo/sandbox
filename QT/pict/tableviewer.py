from PyQt6 import QtCore, QtWidgets
from backend.actions import BaseAction
from backend.state import State

class TableViewer (QtWidgets.QWidget):
    image_selected = QtCore.pyqtSignal(str)

    def __init__(self, parent):
        super().__init__(parent)
        self.main_win = parent
        self.init_main_layout()

        self.proxy_model = QtCore.QSortFilterProxyModel()
        self.proxy_model.setSourceModel(State().model)

    def init_main_layout(self):
        self.table_view = QtWidgets.QTableView()
        self.table_view.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.DoubleClicked)
        self.table_view.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.table_view.setContextMenuPolicy(QtCore.Qt.ContextMenuPolicy.CustomContextMenu)
        self.table_view.customContextMenuRequested.connect(self.show_context_menu)
        self.table_view.setMouseTracking(True)
        self.table_view.viewport().installEventFilter(self)

        self.load_images_button = QtWidgets.QPushButton("Open Folder")
        self.load_images_button.clicked.connect(State().actions["LoadImages"].do)

        State().signals.load_images_signal.connect(self.init_connections)
        State().signals.delete_image_signal.connect(self.update_row_focus)

        self.delete_images_button = QtWidgets.QPushButton("Delete All Images")
        self.delete_images_button.clicked.connect(State().actions["DeleteAllImages"].do)

        self.filter_line_edit = QtWidgets.QLineEdit()
        self.filter_line_edit.setPlaceholderText("Filter by name...")
        self.filter_line_edit.textChanged.connect(self._on_filter_text_changed)

        main_layout = QtWidgets.QVBoxLayout()
        main_layout.addWidget(self.filter_line_edit)
        main_layout.addWidget(self.table_view)
        main_layout.addWidget(self.load_images_button)
        main_layout.addWidget(self.delete_images_button)

        main_layout.setContentsMargins(10, 0, 10, 0)
        table_widget = QtWidgets.QWidget()
        table_widget.setLayout(main_layout)
        self.setLayout(main_layout)

    def show_context_menu(self, position):
        selected_rows = self.table_view.selectionModel().selectedRows()
        context_menu = QtWidgets.QMenu(self.table_view)

        rename_action = State().actions["RenameFile"]
        delete_action = State().actions["DeleteImage"]

        context_menu.addAction(delete_action)
        source_indexes = [self.proxy_model.mapToSource(index) for index in selected_rows]        
        delete_action.triggered.connect(lambda: delete_action.do(source_indexes))

        context_menu.addAction(rename_action)
        source_index = selected_rows[0]
        rename_action.triggered.connect(lambda: rename_action.do(self.proxy_model.mapToSource(source_index)))

        context_menu.exec(self.table_view.viewport().mapToGlobal(position))

        rename_action.triggered.disconnect()
        delete_action.triggered.disconnect()

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key.Key_Delete:
            selected_rows = self.table_view.selectionModel().selectedRows()
            source_indexes = [self.proxy_model.mapToSource(index) for index in selected_rows]
            State().actions["DeleteImage"].do(source_indexes)
        else:
            super().keyPressEvent(event)

    def init_connections(self):
        self.table_view.setModel(self.proxy_model)
        self.table_view.selectionModel().selectionChanged.connect(self.on_selection_changed)
        self.table_view.setSortingEnabled(True)
        for column_index in range(len(State().model.columns)):
            self.table_view.horizontalHeader().setSectionResizeMode(column_index, QtWidgets.QHeaderView.ResizeMode.Stretch)
        self.main_win.show_folder_name(State().current_dir)

    def _on_filter_text_changed(self, text):
        if State().model:
            self.proxy_model.setFilterFixedString(text)

    def on_selection_changed(self, selected, deselected):
        for index in selected.indexes():
            if index.isValid() and index.column() == 0:
                source_index = self.proxy_model.mapToSource(index)
                if source_index.isValid():
                    image_info = State().model.images[source_index.row()]
                    image_name = image_info.name
                    State().selected_image = image_name
                    self.image_selected.emit(State().get_path())
                    self.main_win.show_folder_name(State().current_dir)

    def update_row_focus(self):
        selected = self.table_view.currentIndex()
        current_row = selected.row()

        new_row = min(current_row, self.proxy_model.rowCount() - 1)
        if new_row >= 0:
            new_index = self.proxy_model.index(new_row, 0)

            self.table_view.selectionModel().setCurrentIndex(
                new_index,
                QtCore.QItemSelectionModel.SelectionFlag.ClearAndSelect |
                QtCore.QItemSelectionModel.SelectionFlag.Rows
            )
            self.table_view.scrollTo(new_index)
            self.table_view.setFocus()