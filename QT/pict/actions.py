from PyQt6 import QtCore, QtGui, QtWidgets
import os
from PIL import Image
from qt_common import show_message
from table import ImageInfo

class LoadImagesAction(QtGui.QAction):
    def __init__(self, parent):
        super().__init__("Load Images", parent)
        self.triggered.connect(self.do)
        self.setShortcut(QtGui.QKeySequence("Ctrl+L"))
        self.parent = parent

    def do(self):
        try:
            self.do_impl()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self.parent, "ERROR", str(e))

    def on_selection_changed(self, selected, deselected):
        for index in selected.indexes():
            if index.isValid() and index.column() == 0:
                source_index = self.parent.image_proxy_model.mapToSource(index)
                if source_index.isValid():
                    image_info = self.parent.image_model.images[source_index.row()]
                    image_name = image_info.name
                    image_path = os.path.join(self.parent.image_model.dir_path, image_name)
                    self.parent.image_selected.emit(image_path)

    def do_impl(self):
        folder = QtWidgets.QFileDialog.getExistingDirectory(self.parent, "Select Folder")
        if folder:
            images = []
            for filename in os.listdir(folder):
                if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp')):
                    image_path = os.path.join(folder, filename)
                    try:
                        size = os.path.getsize(image_path)
                        image = Image.open(image_path)
                        width, height = image.size
                        images.append(ImageInfo(filename, size, width, height))
                    except Exception as e:
                        print(f"Error loading image {filename}: {str(e)}")

            if images:
                self.parent.image_model.add_data(images, dir_path=folder)
                self.parent.image_proxy_model.setSourceModel(self.parent.image_model)
                self.parent.table_view.setModel(self.parent.image_proxy_model)
                self.parent.table_view.selectionModel().selectionChanged.connect(self.on_selection_changed)
                self.parent.table_view.setSortingEnabled(True)
                for column_index in range(len(self.parent.image_model.columns)):
                    self.parent.table_view.horizontalHeader().setSectionResizeMode(column_index, QtWidgets.QHeaderView.ResizeMode.Stretch)

class DeleteAllImagesAction(QtGui.QAction):
    def __init__(self, parent):
        super().__init__("Delete All Images", parent)
        self.triggered.connect(self.do)
        self.setShortcut(QtGui.QKeySequence("Ctrl+D"))
        self.parent = parent

    def do(self):
        try:
            self.do_impl()
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "ERROR", str(e))

    def do_impl(self):
        if self.parent.image_model and self.parent.image_model.dir_path:
            reply = QtWidgets.QMessageBox.question(
                self.parent,
                'Delete All Images',
                'Are you sure you want to delete all images?',
                QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
                QtWidgets.QMessageBox.StandardButton.No
            )

            if reply == QtWidgets.QMessageBox.StandardButton.Yes:
                try:
                    for filename in os.listdir(self.parent.image_model.dir_path):
                        file_path = os.path.join(self.parent.image_model.dir_path, filename)
                        if os.path.isfile(file_path) and filename.lower().endswith(('.png', '.jpg', '.jpeg', '.gif', '.bmp')):
                            os.remove(file_path)

                    self.parent.table_view.setModel(None)
                    self.parent.image_selected.emit("")
                    self.parent.filter_line_edit.clear()
                    self.parent.image_model.dir_path = ""

                    show_message(self.parent, "Success", "All images have been deleted.", is_error=False)
                except Exception as e:
                    show_message(self.parent, "Error", f"Error while deleting images: {str(e)}", is_error=True)
        else:
            show_message(self.parent, "Error", "No directory selected.", is_error=True)

class AddImageAction(QtGui.QAction):
    def __init__(self, model, parent=None):
        super().__init__("Add Image", parent)
        self.model = model
        self.triggered.connect(self.do)

    def do(self):
        try:
            self.do_impl()
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "ERROR", str(e))

    def do_impl(self):
        if self.model.dir_path is None:
            QtWidgets.QMessageBox.warning(None, "Error", "Please select a directory first.")
            return
        options = QtWidgets.QFileDialog.Option.DontUseNativeDialog
        file_name, _ = QtWidgets.QFileDialog.getOpenFileName(
            None,
            "Select Image",
            "",
            "Images (*.png *.jpg *.jpeg *.bmp *.gif);;All Files (*)",
            options=options
        )

        if file_name:
            image_name = os.path.basename(file_name)
            image_size = os.path.getsize(file_name)
            with Image.open(file_name) as img:
                width, height = img.size

            new_image_info = ImageInfo(name=image_name, size=image_size, width=width, height=height)
            if not self.model.add_image(new_image_info, file_name):
                QtWidgets.QMessageBox.critical(None, "Error", "Failed to add image.")

class RenameFileAction(QtGui.QAction):
    def __init__(self, model, index, parent=None):
        super().__init__("Rename File", parent)
        self.model = model
        self.index = index
        self.triggered.connect(self.do)

    def do(self):
        try:
            self.do_impl()
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "ERROR", str(e))

    def do_impl(self):
        index = self.index
        if index.isValid():
            old_name = self.model.images[index.row()].name
            new_name, ok = QtWidgets.QInputDialog.getText(None, "Rename File", "Enter new name:", text=old_name)

            if ok and new_name.strip():
                new_name = new_name.strip()
                if new_name != old_name:
                    old_path = os.path.join(self.model.dir_path, old_name)
                    new_path = os.path.join(self.model.dir_path, new_name)

                    if os.path.exists(new_path):
                        QtWidgets.QMessageBox.warning(None, "Error", "File with this name already exists.")
                        return

                    try:
                        os.rename(old_path, new_path)
                        self.model.images[index.row()].name = new_name
                        self.model.dataChanged.emit(index, index, [QtCore.Qt.ItemDataRole.DisplayRole])
                    except Exception as e:
                        QtWidgets.QMessageBox.warning(None, "Error", f"Could not rename file: {e}")