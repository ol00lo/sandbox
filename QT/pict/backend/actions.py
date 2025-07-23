from PyQt6 import QtCore, QtGui, QtWidgets
import os
import sys
import traceback
from qt_common import show_message, SUPPORTED_IMAGE_EXTENSIONS, SUPPORTED_IMAGE_FILTER
from .table import ImageInfo
from .state import State
from .box import Box

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
            return self.do_impl(*args)
        except Exception as e:
            self.errors(e)
    def undo(self, *args):
        try:
            self.undo_impl(*args)
        except Exception as e:
            self.errors(e)
    def redo(self, *args):
        try:
            self.redo_impl(*args)
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
    def undo_impl(self, *args):
        raise NotImplementedError("Subclasses must implement undo_impl()")
    def redo_impl(self, *args):
        raise NotImplementedError("Subclasses must implement redo_impl()")

    def _open_folder(self, folder):
        if not folder:
            return
        images = []
        for filename in os.listdir(folder):
            if filename.lower().endswith(SUPPORTED_IMAGE_EXTENSIONS):
                image_path = os.path.join(folder, filename)
                images.append(ImageInfo(image_path))

        State().model.set_data(images, dir_path=folder)
        State().set_current_dir(folder)
        State().signals.load_images_signal.emit()

class LoadImagesAction(BaseAction):
    _action_name = "LoadImages"

    def __init__(self):
        super().__init__("Ctrl+L")
        self.setIcon(QtGui.QIcon(":/load"))

    def do_impl(self, *args):
        if args:
            folder = args[0]
        else:
            folder = QtWidgets.QFileDialog.getExistingDirectory(None, "Select Folder")
        old_folder = State().current_dir
        self._open_folder(folder)

        return {
            "undo_data": (old_folder,),
            "redo_data": (folder,)
        }

    def undo_impl(self, old_folder):
        if not old_folder: State().cleanup()
        else: State().signals.load_images_signal.emit()

    def redo_impl(self, folder):
        self._open_folder(folder)

class DeleteAllImagesAction(BaseAction):
    _action_name = "DeleteAllImages"
    def __init__(self):
        super().__init__("Ctrl+D")
        self.setIcon(QtGui.QIcon(":/deleteall"))

    def do_impl(self, *args):
        if State().model and State().current_dir is not None:
            deleted_files = self._copy_data(State().current_dir)
            self._delete_data()
        else:
            raise Exception("No directory selected.")

        return {
            'undo_data': (deleted_files,),
            'redo_data': (0,)
        }

    def undo_impl(self, deleted_files):
        if not deleted_files:
            return
        path = deleted_files['directory']
        for image in deleted_files['images']:
            with open(path + "/" + image['name'], 'wb') as f:
                f.write(image['content'])

        self._open_folder(path)

    def redo_impl(self, *args):
        self._delete_data()

    def _copy_data(self, directory):
        data = {
            'directory': directory,
            'csv': [],
            'images': []
        }

        data['csv'] = State().box_saver.get_csv()
        for filename in os.listdir(directory):
            if filename.lower().endswith(SUPPORTED_IMAGE_EXTENSIONS):
                image = {
                    'name': filename,
                    'content': None
                }
                file_path = os.path.join(directory, filename)
                with open(file_path, 'rb') as f:
                    image['content']=f.read()
                data['images'].append(image)

        return data

    def _delete_data(self):
        n_files = 0
        for filename in os.listdir(State().current_dir):
            file_path = os.path.join(State().current_dir, filename)
            if os.path.isfile(file_path) and filename.lower().endswith(SUPPORTED_IMAGE_EXTENSIONS):
                os.remove(file_path)
                n_files += 1

        if n_files > 0:
            State().model.clear_data()
            State().cleanup()
            State().signals.all_images_deleted_signal.emit()

class AddImageAction(BaseAction):
    _action_name = "AddImage"
    def __init__(self):
        super().__init__()
        self.setIcon(QtGui.QIcon(":/add"))

    def do_impl(self, *args):
        if State().current_dir is None:
            QtWidgets.QMessageBox.warning(None, "Error", "Please select a directory first.")
            return
        if args:
            file_path = args[0]
        else:
            options = QtWidgets.QFileDialog.Option.DontUseNativeDialog
            file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
                None,
                "Select Image",
                "",
                f"{SUPPORTED_IMAGE_FILTER};;All Files (*)",
                options=options
            )

        self._add_image(file_path)
        return {
            'undo_data': (os.path.basename(file_path),),
            'redo_data': (file_path,)
        }

    def undo_impl(self, file_name):
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

    def redo_impl(self, file_name):
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

            self._rename(row, old_name, new_name)

            return {
                'undo_data': (index, old_name),
                'redo_data': (index, new_name)
            }

    def undo_impl(self, index, old_name):
        if index.isValid():
            row = index.row()
            curr_name = State().model.images[row].name
            self._rename(row, curr_name, old_name)

    def redo_impl(self, index, new_name):
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

    def do_impl(self, selected):
        if not selected:
            raise Exception("No selection")
        deleted = []
        indexes = []
        for index in sorted(selected, key=lambda x: x.row(), reverse=True):
            out = self.delete(index)
            deleted.append(out)
            indexes.append(index)

        return {
            'undo_data': (deleted,),
            'redo_data': (indexes,)
        }

    def delete(self, index):
        model = State().model
        if not index.isValid():
            return

        current_row = index.row()
        if current_row >= len(model.images) or current_row < 0:
            return

        image_info = model.images[current_row]
        image_path = os.path.join(model.dir_path, image_info.name)

        boxes = State().box_saver.get_boxes_for_image(image_info.name)

        image_data = {
            'info': image_info,
            'content': None,
            'boxes': boxes,
            'path': image_path,
            'row': current_row
        }

        if os.path.exists(image_path):
            with open(image_path, 'rb') as f:
                image_data['content'] = f.read()

        model.beginRemoveRows(QtCore.QModelIndex(), current_row, current_row)
        if os.path.exists(image_path):
            os.remove(image_path)
        model.images.pop(current_row)
        model.endRemoveRows()
        State().box_saver.delete_boxes_on_image(image_info.name)

        return image_data

    def undo_impl(self, deleted):
        for item in deleted:
            if item['content']:
                with open(item['path'], 'wb') as f:
                    f.write(item['content'])

            State().model.beginInsertRows(QtCore.QModelIndex(), item['row'], item['row'])
            State().model.images.insert(item['row'], item['info'])
            State().model.endInsertRows()

            for box in item['boxes']:
                State().box_saver.add_bbox(box, item['info'].name)

    def redo_impl(self, indexes):
        for index in indexes:
            self.delete(index)

class CreateBoxAction(BaseAction):
    _action_name = "CreateBox"
    def __init__(self):
        super().__init__()

    def do_impl(self, box: Box, img_name: str):
        State().box_saver.new_bbox(Box(box.label, box), img_name)
        State().signals.create_box_signal.emit(img_name, box)

        return {
            'undo_data': (img_name, box),
            'redo_data': (img_name, box)
        }

    def undo_impl(self, img_name, box):
        name = os.path.basename(img_name)
        State().box_saver.delete_bbox(box, name)
        State().signals.delete_box_signal.emit(State().current_dir + "/" + name, box)

    def redo_impl(self, img_name, box):
        State().box_saver.new_bbox(Box(box.label, box), img_name)
        State().signals.create_box_signal.emit(img_name, box)

class DeleteBoxAction(BaseAction):
    _action_name = "DeleteBox"
    def __init__(self):
        super().__init__()

    def do_impl(self, box: Box, path: str):
        name = os.path.basename(path)
        ok = State().box_saver.delete_bbox(box, name) 
        if ok: State().signals.delete_box_signal.emit(path, box)

        return {
            'undo_data': (path, box),
            'redo_data': (path, box)
        }

    def undo_impl(self, path, box):
        name = os.path.basename(path)
        State().box_saver.new_bbox(box, name)
        State().signals.create_box_signal.emit(path, box)

    def redo_impl(self, path, box):
        name = os.path.basename(path)
        State().box_saver.delete_bbox(box, name)
        State().signals.delete_box_signal.emit(path, box)

class ResizeBoxAction(BaseAction):
    _action_name = "ResizeBox"
    def __init__(self):
        super().__init__()

    def do_impl(self, old_box: Box, new_box: Box, path):
        name = os.path.basename(path)
        State().box_saver.update_bbox(old_box, new_box, name)
        State().signals.change_boxes_signal.emit(path)

        return {
            'undo_data': (path, old_box, new_box),
            'redo_data': (path, old_box, new_box)
        }

    def undo_impl(self, path, old_box, new_box):
        name = os.path.basename(path)
        State().box_saver.update_bbox(new_box, old_box, name)
        State().signals.change_boxes_signal.emit(path)

    def redo_impl(self, path, old_box, new_box):
        name = os.path.basename(path)
        State().box_saver.update_bbox(old_box, new_box, name)
        State().signals.change_boxes_signal.emit(path)