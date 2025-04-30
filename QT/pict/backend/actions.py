from PyQt6 import QtCore, QtGui, QtWidgets
import os
import sys
import traceback
from qt_common import show_message, SUPPORTED_IMAGE_EXTENSIONS, SUPPORTED_IMAGE_FILTER
from .table import ImageInfo
from .state import State

class BaseAction(QtGui.QAction):
    _action_name = "BaseAction"
    def __init__(self, shortcut=None):
        super().__init__(self._action_name)
        if shortcut:
            self.setShortcut(shortcut)

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        if cls._action_name and cls is not BaseAction:
            State.register_action(cls._action_name, cls)

    def do(self, *args):
        try:
            self.do_impl(*args)
        except Exception as e:
            self.errors(e)

    def errors(self, error):
        exc_type, exc_value, exc_traceback = sys.exc_info()
        tb_str = ''.join(traceback.format_exception(exc_type, exc_value, exc_traceback))
        print("\n=== ERROR TRACEBACK ===")
        print(tb_str)
        print("======================\n")
        show_message("ERROR", str(error), is_error=True)

    def do_impl(self, *args):
        raise NotImplementedError("Subclasses must implement do_impl()")

class LoadImagesAction(BaseAction):
    _action_name = "LoadImages"

    def __init__(self):
        super().__init__("Ctrl+L")
        self.setIcon(QtGui.QIcon(":/load"))

    def do_impl(self, *args):
        folder = QtWidgets.QFileDialog.getExistingDirectory(None, "Select Folder")
        if not folder:
            return
        images = []
        for filename in os.listdir(folder):
            if filename.lower().endswith(SUPPORTED_IMAGE_EXTENSIONS):
                image_path = os.path.join(folder, filename)
                images.append(ImageInfo(image_path))

        State().model.set_data(images, dir_path=folder)
        State().signals.load_images_signal.emit()
        State().current_dir = folder

class DeleteAllImagesAction(BaseAction):
    _action_name = "DeleteAllImages"
    def __init__(self):
        super().__init__("Ctrl+D")
        self.setIcon(QtGui.QIcon(":/deleteall"))

    def do_impl(self, *args):
        if State().model and State().current_dir is not None:
            question = QtWidgets.QMessageBox()
            question.setIcon(QtWidgets.QMessageBox.Icon.Question)
            question.setWindowTitle('Delete All Images')
            question.setText('Are you sure you want to delete all images?')
            question.setStandardButtons(
                 QtWidgets.QMessageBox.StandardButton.Yes | 
                 QtWidgets.QMessageBox.StandardButton.No
            )
            question.setDefaultButton(QtWidgets.QMessageBox.StandardButton.No)
            reply = question.exec()

            if reply == QtWidgets.QMessageBox.StandardButton.Yes:
                deleted_files = 0
                for filename in os.listdir(State().current_dir):
                    file_path = os.path.join(State().current_dir, filename)
                    if os.path.isfile(file_path) and filename.lower().endswith(SUPPORTED_IMAGE_EXTENSIONS):
                        os.remove(file_path)
                        deleted_files += 1

                if deleted_files > 0:
                    State().model.clear_data()
                    State().cleanup()
                    show_message("Success", f"Deleted {deleted_files} images.", is_error=False)

                    State().signals.delete_image_signal.emit()
        else:
            raise Exception("No directory selected.")

class AddImageAction(BaseAction):
    _action_name = "AddImage"
    def __init__(self):
        super().__init__()
        self.setIcon(QtGui.QIcon(":/add"))

    def do_impl(self, *args):
        if State().current_dir is None:
            QtWidgets.QMessageBox.warning(None, "Error", "Please select a directory first.")
            return
        options = QtWidgets.QFileDialog.Option.DontUseNativeDialog
        file_name, _ = QtWidgets.QFileDialog.getOpenFileName(
            None,
            "Select Image",
            "",
            f"{SUPPORTED_IMAGE_FILTER};;All Files (*)",
            options=options
        )

        if file_name:
            new_image_info = ImageInfo(file_name)
            State().model.add_image(new_image_info, file_name)

class RenameFileAction(BaseAction):
    _action_name = "RenameFile"
    def __init__(self):
        super().__init__()
        self.setIcon(QtGui.QIcon(":/rename"))

    def do_impl(self, index):
        if index.isValid():
            row = index.row()

            old_name = State().model.images[row].name
            new_name, ok = QtWidgets.QInputDialog.getText(
                None,
                "Rename File",
                "Enter new name:",
                text=old_name
            )
            new_name = new_name.strip()

            if not ok or not new_name or new_name == old_name:
                return

            old_path = os.path.join(State().current_dir, old_name)
            new_path = os.path.join(State().current_dir, new_name)

            if os.path.exists(new_path):
                QtWidgets.QMessageBox.warning(
                    None,
                    "Error",
                    "File with this name already exists."
                )
                return

            os.rename(old_path, new_path)

            State().model.layoutAboutToBeChanged.emit()
            State().model.images[row].name = new_name
            State().model.layoutChanged.emit()

            State().signals.rename_image_signal.emit(new_path)

class DeleteImageAction(BaseAction):
    _action_name = "DeleteImage"
    def __init__(self):
        super().__init__("Delete")
        self.setIcon(QtGui.QIcon(":/delete"))

    def do_impl(self, selected):
        if not selected:
            raise Exception("No selection")
        for index in sorted(selected, key=lambda x: x.row(), reverse=True):  
            self.delete(index)
            State().signals.delete_image_signal.emit()

    def delete(self, index):
        model = State().model
        if not index.isValid():
            return

        current_row = index.row()
        if current_row >= len(model.images) or current_row < 0:
            return

        image_info = model.images[current_row]
        image_path = os.path.join(model.dir_path, image_info.name)

        model.beginRemoveRows(QtCore.QModelIndex(), current_row, current_row)
        if os.path.exists(image_path):
            os.remove(image_path)
        model.images.pop(current_row)
        model.endRemoveRows()