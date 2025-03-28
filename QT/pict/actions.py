from PyQt6 import QtCore, QtGui, QtWidgets
import os
from PIL import Image
from table import TableModel, TableProxyModel, ImageInfo
from qt_common import show_message

class TableActions(QtCore.QObject):
    def __init__(self, parent):
        super().__init__()
        self.table_view = parent.table_view
        self.parent = parent
        self.parent.image_selected.emit("")
        self.table_view.setContextMenuPolicy(QtCore.Qt.ContextMenuPolicy.CustomContextMenu)
        self.table_view.customContextMenuRequested.connect(self.show_context_menu)

    def on_filter_text_changed(self, text):
        print("Filter text changed:", text)
        if self.image_model:
            self.image_proxy_model.setFilterFixedString(text)

    def load_images(self):
        folder = QtWidgets.QFileDialog.getExistingDirectory(self.parent, "Select Folder")

        if folder:
            images = []
            for filename in os.listdir(folder):
                if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif')):
                    image_path = os.path.join(folder, filename)
                    size = os.path.getsize(image_path)
                    image = Image.open(image_path)
                    width, height = image.size
                    images.append(ImageInfo(filename, size, width, height))
            if images:
                self.image_model = TableModel(images, dir_path=folder)
                self.image_proxy_model = TableProxyModel(self.parent)
                self.image_proxy_model.setSourceModel(self.image_model)

                self.table_view.setModel(self.image_proxy_model)
                self.table_view.selectionModel().selectionChanged.connect(self.on_selection_changed)

                self.table_view.setSortingEnabled(True)
                for column_index in range(len(self.image_model.columns)):
                    self.table_view.horizontalHeader().setSectionResizeMode(column_index, QtWidgets.QHeaderView.ResizeMode.Stretch)


    def delete_all_images(self):
        if self.image_model and self.image_model.dir_path:
            reply = QtWidgets.QMessageBox.question(
                self.parent,
                'Delete All Images',
                'Are you sure you want to delete all images?',
                QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
                QtWidgets.QMessageBox.StandardButton.No
            )

            if reply == QtWidgets.QMessageBox.StandardButton.Yes:
                try:
                    for filename in os.listdir(self.image_model.dir_path):
                        file_path = os.path.join(self.image_model.dir_path, filename)
                        if os.path.isfile(file_path) and filename.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp')):
                            os.remove(file_path)

                    self.parent.table_view.setModel(None)
                    self.parent.image_viewer.display_image("")
                    self.parent.filter_line_edit.clear()
                    self.image_model.dir_path = ""

                    show_message(self.parent, "Success", "All images have been deleted.", is_error=False)
                except Exception as e:
                    show_message(self.parent, "Error", f"Error while deleting images: {str(e)}", is_error=True)
        else:
            show_message(self.parent, "Error", "No directory selected.", is_error=True)

    def on_selection_changed(self, selected, deselected):
        for index in selected.indexes():
            if index.isValid() and index.column() == 0:
                source_index = self.image_proxy_model.mapToSource(index)
                if source_index.isValid():
                    image_info = self.image_model.images[source_index.row()]
                    self.display_image(image_info.name)

    def display_image(self, image_name):
        image_path = os.path.join(self.image_model.dir_path, image_name)
        self.parent.image_selected.emit(image_path)

    def show_context_menu(self, position):
        index = self.table_view.indexAt(position)
        if index.isValid():
            context_menu = QtWidgets.QMenu(self.table_view)
            delete_action = context_menu.addAction("Delete Image")
            delete_action.triggered.connect(lambda: self.delete_image(index))

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
            except Exception as e:
                show_message(self.parent, "Error", f"Error while deleting image: {str(e)}", is_error=True)
