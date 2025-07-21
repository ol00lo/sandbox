from PyQt6 import QtCore, QtGui, QtWidgets
import os
import sys
import traceback
import csv
import shutil
from pathlib import Path
from qt_common import show_message, SUPPORTED_IMAGE_EXTENSIONS, SUPPORTED_IMAGE_FILTER
from .table import ImageInfo
from .state import State
from .box import Box
from .commands import Command, ActionResult

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

    def do_impl(self, folder = None):
        if not folder:
            folder = QtWidgets.QFileDialog.getExistingDirectory(None, "Select Folder")
        old_folder = State().current_dir
        self._open_folder(folder)

        return ActionResult([old_folder], [folder])

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
        return ActionResult([deleted_files], [0])

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

    def do_impl(self, file_path=None):
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
            return ActionResult([index, old_name], [index, new_name])

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

        backup_dir = Path(State().backup_dir) / State().current_dir.split("/")[-2]
        backup_dir.mkdir(parents=True, exist_ok=True)
        deleted = []

        sorted_indexes = sorted(selected, key=lambda x: x.row(), reverse=True)

        for index in sorted_indexes:
            deleted.append(self.delete(index, backup_dir))

        return ActionResult([deleted], [sorted_indexes])

    def delete(self, index, backup_dir=None):
        model = State().model
        if not index.isValid() or not (0 <= index.row() < len(model.images)):
            return None

        image_info = model.images[index.row()]
        image_path = Path(model.dir_path) / image_info.name
        boxes = State().box_saver.get_boxes_for_image(image_info.name)

        image_data = {
            'info': image_info,
            'path': str(image_path),
            'backup_path': str(backup_dir),
            'row': index.row()
        }

        if image_path.exists():
            backup_image_path = backup_dir / image_info.name

            backup_csv_path = backup_dir / f"{image_info.name.split('.')[0]}.csv"

            shutil.copy2(image_path, backup_image_path)

            with backup_csv_path.open('w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerows(box.to_csv_row() for box in boxes)

        model.beginRemoveRows(QtCore.QModelIndex(), index.row(), index.row())
        if os.path.exists(image_path):
            os.remove(image_path)
        model.images.pop(index.row())
        model.endRemoveRows()
        State().box_saver.delete_boxes_on_image(image_info.name)

        return image_data

    def undo_impl(self, deleted):
        model = State().model

        for item in deleted:
            backup_image_path = Path(item['backup_path']) / item['info'].name
            original_path = Path(item['path'])

            if backup_image_path.exists():
                shutil.copy2(backup_image_path, original_path)

                backup_csv_path = backup_image_path.with_suffix('.csv')
                if backup_csv_path.exists():
                    with open(backup_csv_path, newline='') as csvfile:
                        reader = csv.reader(csvfile)
                        for row in reader:
                            label, x1, y1, x2, y2 = row
                            box = Box(label, QtCore.QRectF( float(x1), float(y1), float(x2), float(y2)))
                            State().box_saver.add_bbox(box, item['info'].name)

            model.beginInsertRows(QtCore.QModelIndex(), item['row'], item['row'])
            model.images.insert(item['row'], item['info'])
            model.endInsertRows()
            model.dataChanged.emit(model.index_by_imagename(item['info'].name), model.index_by_imagename(item['info'].name))


    def redo_impl(self, indexes):
        backup_dir = Path(State().backup_dir) / State().current_dir.split("/")[-2]
        deleted = []

        for index in indexes:
            deleted.append(self.delete(index, backup_dir))

        return deleted

class CreateBoxAction(BaseAction):
    _action_name = "CreateBox"
    def __init__(self):
        super().__init__()

    def do_impl(self, box: Box, img_name: str):
        State().box_saver.new_bbox(Box(box.label, box), img_name)
        State().signals.create_box_signal.emit(img_name, box)
        return ActionResult([img_name, box], [img_name, box])

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
        return ActionResult([path, box], [path, box])

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
        return ActionResult([path, old_box, new_box], [path, old_box, new_box])

    def undo_impl(self, path, old_box, new_box):
        name = os.path.basename(path)
        State().box_saver.update_bbox(new_box, old_box, name)
        State().signals.change_boxes_signal.emit(path)

    def redo_impl(self, path, old_box, new_box):
        name = os.path.basename(path)
        State().box_saver.update_bbox(old_box, new_box, name)
        State().signals.change_boxes_signal.emit(path)