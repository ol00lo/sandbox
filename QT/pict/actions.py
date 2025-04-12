from PyQt6 import QtCore, QtGui, QtWidgets
import os
import sys
import traceback
from qt_common import show_message, SUPPORTED_IMAGE_EXTENSIONS, SUPPORTED_IMAGE_FILTER
from table import ImageInfo
from state import State

class BaseAction(QtGui.QAction):
    _action_name = None  

    def __init__(self, parent, shortcut=None):
        super().__init__(self._action_name, parent)
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
    def __init__(self, main_win):
        super().__init__(main_win, "Ctrl+L")
        self.main_win = main_win
        self.setIcon(QtGui.QIcon(":/load"))

    def on_selection_changed(self, selected, deselected):
        for index in selected.indexes():
            if index.isValid() and index.column() == 0:
                source_index = State().proxy_model.mapToSource(index)
                if source_index.isValid():
                    image_info = State().model.images[source_index.row()]
                    image_name = image_info.name
                    State().selected_image = image_name
                    self.main_win.image_selected.emit(State().get_path())
                    self.main_win.curr_dir_signal.emit(State().current_dir)

    def do_impl(self, *args):
        folder = QtWidgets.QFileDialog.getExistingDirectory(self.main_win, "Select Folder")
        if not folder:
            return
        images = []
        for filename in os.listdir(folder):
            if filename.lower().endswith(SUPPORTED_IMAGE_EXTENSIONS):
                image_path = os.path.join(folder, filename)
                images.append(ImageInfo(image_path))

        if images:
            State().model.set_data(images, dir_path=folder)
            self.main_win.load_images_signal.emit()
        State().current_dir = folder
        self.main_win.curr_dir_signal.emit(folder)


class DeleteAllImagesAction(BaseAction):
    _action_name = "DeleteAllImages"
    def __init__(self, main_win):
        super().__init__(main_win, "Ctrl+D")
        self.setIcon(QtGui.QIcon(":/deleteall"))
        self.main_win = main_win

    def do_impl(self, *args):
        if State().model and State().current_dir is not None:
            reply = QtWidgets.QMessageBox.question(
                self.main_win,
                'Delete All Images',
                'Are you sure you want to delete all images?',
                QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No,
                QtWidgets.QMessageBox.StandardButton.No
            )

            if reply == QtWidgets.QMessageBox.StandardButton.Yes:
                deleted_files = 0
                for filename in os.listdir(State().current_dir):
                    file_path = os.path.join(State().current_dir, filename)
                    if os.path.isfile(file_path) and filename.lower().endswith(SUPPORTED_IMAGE_EXTENSIONS):
                        os.remove(file_path)
                        deleted_files += 1

                if deleted_files > 0:
                    State().model.beginResetModel()
                    State().model.images.clear()
                    State().model.dir_path = ""
                    State().model.endResetModel()

                    State().cleanup()
                    self.main_win.image_selected.emit(State().get_path())
                    show_message("Success", f"Deleted {deleted_files} images.", is_error=False)
        else:
            show_message("Error", "No directory selected.", is_error=True)
            return

class AddImageAction(BaseAction):
    _action_name = "AddImage"
    def __init__(self,  main_win):
        super().__init__(main_win)
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

    def __init__(self, main_win):
        super().__init__(main_win)
        self.setIcon(QtGui.QIcon(":/rename"))
        self.main_win = main_win

    def do_impl(self, index):
        if index.isValid():
            source_index = State().proxy_model.mapToSource(index)
            row = source_index.row()

            old_name = State().model.images[row].name
            new_name, ok = QtWidgets.QInputDialog.getText(
                self.main_win,
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
                    self.main_win,
                    "Error",
                    "File with this name already exists."
                )
                return

            os.rename(old_path, new_path)

            State().model.layoutAboutToBeChanged.emit()
            State().model.images[row].name = new_name
            State().model.dataChanged.emit(index, index, [QtCore.Qt.ItemDataRole.DisplayRole])
            State().model.layoutChanged.emit()

            if State().selected_image == old_name:
                State().selected_image = new_name
                self.main_win.image_selected.emit(new_path)

class DeleteImageAction(BaseAction):
    _action_name = "DeleteImage"
    def __init__(self, main_win):
        super().__init__(main_win, "Delete")
        self.setIcon(QtGui.QIcon(":/delete"))
        self.main_win = main_win

    def do_impl(self, selected):
        if not selected:
            show_message("No selection", "Please select images to delete", is_error=True)
            return
        for index in sorted(selected, key=lambda x: x.row(), reverse=True):
            self.delete(index)

    def delete(self, index):
        model = State().model
        if not index.isValid():
            return
        source_index = State().proxy_model.mapToSource(index)
        if not source_index.isValid():
            return
        current_row = source_index.row()
        if current_row >= len(model.images) or current_row < 0:
            return

        image_info = model.images[current_row]
        image_path = os.path.join(model.dir_path, image_info.name)

        if os.path.exists(image_path):
            os.remove(image_path)

        model.beginRemoveRows(QtCore.QModelIndex(), current_row, current_row)
        model.images.pop(current_row)
        model.endRemoveRows()

        next_image_path = ""
        if model.images:
            next_row = min(current_row, len(model.images) - 1)
            next_image_info = model.images[next_row]
            next_image_path = os.path.join(model.dir_path, next_image_info.name)

        State().selected_image = next_image_path

        self.main_win.image_selected.emit(next_image_path)