from PyQt6 import QtCore, QtGui, QtWidgets
import os
import sys
import traceback
from PIL import Image
from qt_common import show_message, SUPPORTED_IMAGE_EXTENSIONS
from table import ImageInfo

class BaseAction(QtGui.QAction):
    def __init__(self, name, parent, shortcut=None):
        super().__init__(name, parent)
        self.triggered.connect(self.do)
        if shortcut:
            self.setShortcut(QtGui.QKeySequence(shortcut))
        self.parent = parent

    def do(self):
        try:
            self.do_impl()
        except Exception as e:
            self.errors(e)

    def errors(self, error):
        exc_type, exc_value, exc_traceback = sys.exc_info()
        tb_str = ''.join(traceback.format_exception(exc_type, exc_value, exc_traceback))
        print("\n=== ERROR TRACEBACK ===")
        print(tb_str)
        print("======================\n")
        parent = self.parent if hasattr(self, 'parent') and self.parent else None
        QtWidgets.QMessageBox.critical(parent, "ERROR", str(error))
    
    def do_impl(self):
        raise NotImplementedError("Subclasses must implement do_impl()")

class LoadImagesAction(BaseAction):
    def __init__(self, parent):
        super().__init__("Load Images", parent, "Ctrl+L")

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
                if filename.lower().endswith(SUPPORTED_IMAGE_EXTENSIONS):
                    image_path = os.path.join(folder, filename)
                    try:
                        size = os.path.getsize(image_path)
                        image = Image.open(image_path)
                        width, height = image.size
                        images.append(ImageInfo(filename, size, width, height))
                    except Exception as e:
                        print(f"Error loading image {filename}: {str(e)}")

            if images:
                self.parent.image_model.set_data(images, dir_path=folder)
                self.parent.table_view.setModel(self.parent.image_proxy_model)
                self.parent.table_view.selectionModel().selectionChanged.connect(self.on_selection_changed)
                self.parent.table_view.setSortingEnabled(True)
                for column_index in range(len(self.parent.image_model.columns)):
                    self.parent.table_view.horizontalHeader().setSectionResizeMode(column_index, QtWidgets.QHeaderView.ResizeMode.Stretch)

class DeleteAllImagesAction(BaseAction):
    def __init__(self, parent):
        super().__init__("Delete All Images", parent, "Ctrl+D")

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
                deleted_files = 0
                for filename in os.listdir(self.parent.image_model.dir_path):
                    file_path = os.path.join(self.parent.image_model.dir_path, filename)
                    if os.path.isfile(file_path) and filename.lower().endswith(SUPPORTED_IMAGE_EXTENSIONS):
                        os.remove(file_path)
                        deleted_files += 1

                if deleted_files > 0:
                    self.parent.image_model.beginResetModel()
                    self.parent.image_model.images.clear()
                    self.parent.image_model.dir_path = ""
                    self.parent.image_model.endResetModel()

                    self.parent.image_selected.emit("")
                    self.parent.filter_line_edit.clear()
                    show_message(self.parent, "Success", f"Deleted {deleted_files} images.", is_error=False)
        else:
            show_message(self.parent, "Error", "No directory selected.", is_error=True)
            return


class AddImageAction(BaseAction):
    def __init__(self, model, parent=None):
        super().__init__("Add Image", parent)
        self.model = model

    def do_impl(self):
        if self.model.dir_path is None:
            QtWidgets.QMessageBox.warning(None, "Error", "Please select a directory first.")
            return
        options = QtWidgets.QFileDialog.Option.DontUseNativeDialog
        file_name, _ = QtWidgets.QFileDialog.getOpenFileName(
            None,
            "Select Image",
            "",
            "Images (*.png *.jpg *.jpeg *.bmp);;All Files (*)",
            options=options
        )

        if file_name:
            image_name = os.path.basename(file_name)
            image_size = os.path.getsize(file_name)
            with Image.open(file_name) as img:
                width, height = img.size

            new_image_info = ImageInfo(name=image_name, size=image_size, width=width, height=height)
            self.model.add_image(new_image_info, file_name)

class RenameFileAction(BaseAction):
    def __init__(self, model, index, parent=None):
        super().__init__("Rename File", parent)
        self.model = model
        self.index = index

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

                    os.rename(old_path, new_path)
                    self.model.images[index.row()].name = new_name
                    self.model.dataChanged.emit(index, index, [QtCore.Qt.ItemDataRole.DisplayRole])
