from PyQt6 import QtCore, QtGui, QtWidgets
import os
import sys
import traceback
from pathlib import Path
from qt_common import show_message, SUPPORTED_IMAGE_EXTENSIONS, SUPPORTED_IMAGE_FILTER
from .table import ImageInfo
from .state import State
from .box import Box
from .commands import ActionResult, Command
from drawstate import DrawState

class BaseAction(QtGui.QAction):
    _action_name = "BaseAction"
    def __init__(self, shortcut=None):
        super().__init__(self._action_name)
        if shortcut:
            self.setShortcut(shortcut)
        self.triggered.connect(self._handle_trigger)

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        if cls._action_name and cls is not BaseAction:
            State.register_action(cls._action_name, cls)

    def do(self, *args):
        try:
            return self._do_impl(*args)
        except Exception as e:
            self.errors(e)
            raise e

    def undo(self, *args):
        try:
            self._undo_impl(*args)
        except Exception as e:
            self.errors(e)
            raise e

    def redo(self, *args):
        try:
            self._redo_impl(*args)
        except Exception as e:
            self.errors(e)
            raise e

    def errors(self, error):
        exc_type, exc_value, exc_traceback = sys.exc_info()
        tb_str = ''.join(traceback.format_exception(exc_type, exc_value, exc_traceback))
        print("\n=== ERROR TRACEBACK ===")
        print(tb_str)
        print("======================\n")
        show_message("ERROR", str(error), is_error=True)

    def _handle_trigger(self):
        args = self._get_default_args()
        if args is not None:
            self.execute(*args)

    def execute(self, *args):
        try:
            command = Command(self, *args)
            State().undo_redo_manager.execute(command)
        except Exception as e:
            self.errors(e)

    def _get_default_args(self):
        return ()

    def _do_impl(self, *args):
        raise NotImplementedError("Subclasses must implement do_impl()")

    def _undo_impl(self, *args):
        raise NotImplementedError("Subclasses must implement undo_impl()")

    def _redo_impl(self, *args):
        raise NotImplementedError("Subclasses must implement redo_impl()")

    def _open_folder(self, folder):
        if not folder:
            return
        images = []
        for filename in os.listdir(folder):
            if filename.lower().endswith(SUPPORTED_IMAGE_EXTENSIONS):
                image_path = os.path.join(folder, filename)
                images.append(ImageInfo(image_path))

        State().undo_redo_manager.clear_stack()
        State().model.set_data(images, dir_path=folder)
        State().set_current_dir(folder)
        State().signals.load_images_signal.emit()

class LoadImagesAction(BaseAction):
    _action_name = "LoadImages"

    def __init__(self):
        super().__init__("Ctrl+L")
        self.setIcon(QtGui.QIcon(":/load"))

    def _do_impl(self, folder = None):
        if not folder:
            folder = QtWidgets.QFileDialog.getExistingDirectory(None, "Select Folder")
        old_folder = State().current_dir
        self._open_folder(folder)

        return ActionResult([old_folder], [folder])

    def _undo_impl(self, old_folder):
        if not old_folder: State().cleanup()
        else: State().signals.load_images_signal.emit()

    def _redo_impl(self, folder):
        self._open_folder(folder)

class DeleteAllImagesAction(BaseAction):
    _action_name = "DeleteAllImages"
    def __init__(self):
        super().__init__("Ctrl+D")
        self.setIcon(QtGui.QIcon(":/deleteall"))

    def _do_impl(self, *args):
        if State().model and State().current_dir is not None:
            deleted_info = {
                'directory': State().current_dir,
                'images': []
            }

            for filename in os.listdir(State().current_dir):
                if filename.lower().endswith(SUPPORTED_IMAGE_EXTENSIONS):
                    image_path = os.path.join(State().current_dir, filename)
                    boxes = State().box_saver.get_boxes_for_image(filename)

                    State().backup_manager.save_file(image_path, boxes)
                    deleted_info['images'].append({
                        'name': filename,
                    })
            if deleted_info['images']:
                State().model.clear_data()
                State().cleanup()
                State().signals.all_images_deleted_signal.emit()
            return ActionResult([deleted_info], [0])

    def _undo_impl(self, deleted_info):
        if not deleted_info:
            return

        for image in deleted_info['images']:
            dest_path = os.path.join(deleted_info['directory'], image['name'])
            State().backup_manager.return_file(image['name'], dest_path)

            boxes = State().backup_manager.get_boxes_on_image(image['name'])
            for box in boxes:
                State().box_saver.new_bbox(box, image['name'])

        self._open_folder(deleted_info['directory'])

    def _redo_impl(self, *args):
        self._do_impl()

class AddImageAction(BaseAction):
    _action_name = "AddImage"
    def __init__(self):
        super().__init__()
        self.setIcon(QtGui.QIcon(":/add"))

    def _do_impl(self, file_path=None):
        if State().current_dir is None:
            QtWidgets.QMessageBox.warning(None, "Error", "Please select a directory first.")
            return
        if not file_path:
            options = QtWidgets.QFileDialog.Option.DontUseNativeDialog
            file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
                None,
                "Select Image",
                "",
                f"{SUPPORTED_IMAGE_FILTER};;All Files (*)",
                options=options
            )

        self._add_image(file_path)
        return ActionResult([os.path.basename(file_path)], [file_path])

    def _undo_impl(self, file_name):
        if not file_name:
            return None

        model = State().model
        image_path = os.path.join(model.dir_path, file_name)

        current_row = model.index_by_imagename(file_name).row()

        model.beginRemoveRows(QtCore.QModelIndex(), current_row, current_row)
        if os.path.exists(image_path):
            os.remove(image_path)
        model.images.pop(current_row)
        model.endRemoveRows()

        State().box_saver.delete_boxes_on_image(file_name)

    def _redo_impl(self, file_name):
        self._add_image(file_name)

    def _add_image(self, file_name):
        if file_name:
            new_image_info = ImageInfo(file_name)
            State().model.add_image(new_image_info, file_name)

class RenameFileAction(BaseAction):
    _action_name = "RenameFile"
    def __init__(self):
        super().__init__()
        self.setIcon(QtGui.QIcon(":/rename"))

    def _do_impl(self, index):
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

            self._rename(row, old_name, new_name)
            return ActionResult([index, old_name], [index, new_name])

    def _undo_impl(self, index, old_name):
        if index.isValid():
            row = index.row()
            curr_name = State().model.images[row].name
            self._rename(row, curr_name, old_name)

    def _redo_impl(self, index, new_name):
        if index.isValid():
            row = index.row()
            old_name = State().model.images[row].name
            self._rename(row, old_name, new_name)

    def _rename(self, row, from_name, to_name):
        from_path = os.path.join(State().current_dir, from_name)
        to_path = os.path.join(State().current_dir, to_name)
        if os.path.exists(to_path):
                QtWidgets.QMessageBox.warning(
                    None,
                    "Error",
                    "File with this name already exists."
                )
                return
        os.rename(from_path, to_path)

        State().model.layoutAboutToBeChanged.emit()
        State().model.images[row].name = to_name
        State().model.layoutChanged.emit()
        State().signals.rename_image_signal.emit(to_path)

class DeleteImageAction(BaseAction):
    _action_name = "DeleteImage"
    def __init__(self):
        super().__init__("Delete")
        self.setIcon(QtGui.QIcon(":/delete"))

    def _do_impl(self, selected):
        if not selected:
            raise Exception("No selection")

        deleted = []
        sorted_indexes = sorted(selected, key=lambda x: x.row(), reverse=True)

        for index in sorted_indexes:
            if not index.isValid() or not (0 <= index.row() < len(State().model.images)):
                continue
            image_name = State().model.images[index.row()].name
            image_path = os.path.join(State().model.dir_path, image_name)
            boxes = State().box_saver.get_boxes_for_image(image_name)

            backup_data = State().backup_manager.save_file(image_path, boxes)
            deleted.append(self.delete(index.row()))

        return ActionResult([deleted], [sorted_indexes], backup_data)

    def delete(self, index):
        model = State().model

        image_info = model.images[index]
        image_path = Path(model.dir_path) / image_info.name

        image_data = {
            'info': image_info,
            'path': str(image_path),
            'row': index
        }

        model.beginRemoveRows(QtCore.QModelIndex(), index, index)
        model.images.pop(index)
        model.endRemoveRows()
        State().box_saver.delete_boxes_on_image(image_info.name)

        return image_data

    def _undo_impl(self, deleted):
        model = State().model
        for item in reversed(deleted):
            State().backup_manager.return_file(item['info'].name, item['path'])
            boxes = State().backup_manager.get_boxes_on_image(item['info'].name)
            for box in boxes:
                State().box_saver.new_bbox(box, item['info'].name)

            model.beginInsertRows(QtCore.QModelIndex(), item['row'], item['row'])
            model.images.insert(item['row'], item['info'])
            model.endInsertRows()
            model.dataChanged.emit(model.index_by_imagename(item['info'].name), model.index_by_imagename(item['info'].name))

            row = model.rowCount() - item['row'] - 1
            DrawState().signals.change_focus_signal.emit(0, 0, row)

    def _redo_impl(self, indexes):
        self._do_impl(indexes)

class CreateBoxAction(BaseAction):
    _action_name = "CreateBox"
    def __init__(self):
        super().__init__()

    def _do_impl(self, box: Box, img_name: str):
        State().box_saver.new_bbox(Box(box.label, box), img_name)
        State().signals.create_box_signal.emit(img_name, box)
        return ActionResult([img_name, box], [img_name, box])

    def _undo_impl(self, img_name, box):
        name = os.path.basename(img_name)
        State().box_saver.delete_bbox(box, name)
        State().signals.delete_box_signal.emit(os.path.join(State().current_dir, name), box)

    def _redo_impl(self, img_name, box):
        State().box_saver.new_bbox(Box(box.label, box), img_name)
        State().signals.create_box_signal.emit(img_name, box)

class DeleteBoxAction(BaseAction):
    _action_name = "DeleteBox"
    def __init__(self):
        super().__init__()

    def _do_impl(self, box: Box, path: str):
        name = os.path.basename(path)
        ok = State().box_saver.delete_bbox(box, name) 
        if ok: State().signals.delete_box_signal.emit(path, box)
        return ActionResult([path, box], [path, box])

    def _undo_impl(self, path, box):
        name = os.path.basename(path)
        State().box_saver.new_bbox(box, name)
        State().signals.create_box_signal.emit(path, box)

    def _redo_impl(self, path, box):
        name = os.path.basename(path)
        State().box_saver.delete_bbox(box, name)
        State().signals.delete_box_signal.emit(path, box)

class ResizeBoxAction(BaseAction):
    _action_name = "ResizeBox"
    def __init__(self):
        super().__init__()

    def _do_impl(self, old_box: Box, new_box: Box, path):
        name = os.path.basename(path)
        State().box_saver.update_bbox(old_box, new_box, name)
        State().signals.change_boxes_signal.emit(path)
        return ActionResult([path, old_box, new_box], [path, old_box, new_box])

    def _undo_impl(self, path, old_box, new_box):
        name = os.path.basename(path)
        State().box_saver.update_bbox(new_box, old_box, name)
        State().signals.change_boxes_signal.emit(path)

    def _redo_impl(self, path, old_box, new_box):
        name = os.path.basename(path)
        State().box_saver.update_bbox(old_box, new_box, name)
        State().signals.change_boxes_signal.emit(path)