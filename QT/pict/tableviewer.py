from PyQt6 import QtCore, QtWidgets
from actions import LoadImagesAction, DeleteAllImagesAction, RenameFileAction
from table import TableModel
import os
from qt_common import show_message

class TableViewer (QtWidgets.QWidget):
    image_selected = QtCore.pyqtSignal(str)

    def __init__(self, parent):
        super().__init__(parent)

        self.table_view = QtWidgets.QTableView()
        self.table_view.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.DoubleClicked)
        self.table_view.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.table_view.setContextMenuPolicy(QtCore.Qt.ContextMenuPolicy.CustomContextMenu)
        self.table_view.customContextMenuRequested.connect(self.show_context_menu)
        self.table_view.setMouseTracking(True)
        self.table_view.viewport().installEventFilter(self)

        self.image_model = TableModel(self)
        self.image_proxy_model = QtCore.QSortFilterProxyModel(self)
        self.image_proxy_model.setSourceModel(self.image_model)

        self.filter_line_edit = QtWidgets.QLineEdit()
        self.filter_line_edit.setPlaceholderText("Filter by name...")
        self.filter_line_edit.textChanged.connect(self.on_filter_text_changed)

        self.load_images_button = QtWidgets.QPushButton("Load Images")
        self.load_images_button.clicked.connect(LoadImagesAction(self).do)

        self.delete_images_button = QtWidgets.QPushButton("Delete All Images")
        self.delete_images_button.clicked.connect(DeleteAllImagesAction(self).do)

        main_layout = QtWidgets.QVBoxLayout()
        main_layout.addWidget(self.filter_line_edit)
        main_layout.addWidget(self.table_view)
        main_layout.addWidget(self.load_images_button)
        main_layout.addWidget(self.delete_images_button)

        table_widget = QtWidgets.QWidget()
        table_widget.setMaximumWidth(400)
        table_widget.setLayout(main_layout)
        self.setLayout(main_layout)

    def on_filter_text_changed(self, text):
        if self.image_model:
            self.image_proxy_model.setFilterFixedString(text)

    def show_context_menu(self, position):
        index = self.table_view.indexAt(position)
        if index.isValid():
            context_menu = QtWidgets.QMenu(self.table_view)
            delete_action = context_menu.addAction("Delete Image")
            delete_action.triggered.connect(lambda: self.delete_image(index))

            rename_action = context_menu.addAction("Rename Image")
            rename_action.triggered.connect(lambda: RenameFileAction(self.image_model, index).do())

            context_menu.exec(self.table_view.viewport().mapToGlobal(position))

    def delete_image(self, index):
        source_index = self.image_proxy_model.mapToSource(index)
        if source_index.isValid():
            image_info = self.image_model.images[source_index.row()]
            image_path = os.path.join(self.image_model.dir_path, image_info.name)

            try:
                os.remove(image_path)
                self.image_model.images.pop(source_index.row())
                self.image_model.layoutChanged.emit()

                self.image_selected.emit("")
                self.filter_line_edit.clear()
            except Exception as e:
                show_message(self.parent, "Error", f"Error while deleting image: {str(e)}", is_error=True)