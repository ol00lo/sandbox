from PyQt6 import QtCore, QtGui, QtWidgets
import os
import cv2
from table import TableModel, TableProxyModel, ImageInfo

class TableActions(QtCore.QObject):
    def __init__(self, parent):
        super().__init__()
        self.table_view = parent.table_view
        self.parent = parent
        self.parent.image_selected.emit("")

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

                    image = cv2.imread(image_path)
                    size = image.size
                    height, width, _ = image.shape
                    images.append(ImageInfo(filename, size, width, height))
            if images:
                self.image_model = TableModel(images, dir_path=folder)
                self.image_proxy_model = TableProxyModel(self.parent)
                self.image_proxy_model.setSourceModel(self.image_model)

                self.table_view.setModel(self.image_proxy_model)
                self.table_view.selectionModel().selectionChanged.connect(self.on_selection_changed)

                for column_index in range(len(self.image_model.columns)):
                    self.table_view.horizontalHeader().setSectionResizeMode(column_index, QtWidgets.QHeaderView.ResizeMode.Stretch)

                self.table_view.horizontalHeader().sectionDoubleClicked.connect(self.sort_table)

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

                    self.show_message("Success", "All images have been deleted.", is_error=False)
                except Exception as e:
                    self.show_message("Error", f"Error while deleting images: {str(e)}", is_error=True)
        else:
            self.show_message("Error", "No directory selected.", is_error=True)

    def show_message(self, title, message, is_error=False):
        dialog = QtWidgets.QMessageBox(self.parent)
        dialog.setWindowTitle(title)
        dialog.setText(message)

        if is_error:
            dialog.setWindowIcon(QtGui.QIcon(":/error"))
        else:
            dialog.setWindowIcon(QtGui.QIcon(":/right"))

        dialog.setStandardButtons(QtWidgets.QMessageBox.StandardButton.Ok)
        dialog.exec()

    def on_selection_changed(self, selected, deselected):
        index = self.parent.table_view.selectionModel().currentIndex()
        if index.isValid() and index.column() == 0:
            image_info = self.image_model.images[index.row()]
            self.display_image(image_info.name)

    def sort_table(self, index):
        self.image_proxy_model.sort(index)

    def display_image(self, image_name):
        image_path = os.path.join(self.image_model.dir_path, image_name)
        self.parent.image_selected.emit(image_path)