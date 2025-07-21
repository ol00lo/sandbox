from .table import TableModel
import os
from PyQt6 import QtCore
from .bboxlist import BBoxList
from .commands import Command, UndoRedoManager

class Signals(QtCore.QObject):
    rename_image_signal = QtCore.pyqtSignal(str)
    all_images_deleted_signal =  QtCore.pyqtSignal()
    load_images_signal = QtCore.pyqtSignal()

    create_box_signal = QtCore.pyqtSignal(str, QtCore.QRectF)
    delete_box_signal = QtCore.pyqtSignal(str, QtCore.QRectF)
    change_boxes_signal = QtCore.pyqtSignal(str)

class State:
    _instance = None
    signals = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._init_state()
        return cls._instance

    def _init_state(self):
        self.signals = Signals()
        self.model = TableModel()
        self.signals.change_boxes_signal.connect(self.model.update_boxes)
        self.signals.delete_box_signal.connect(self.model.delete_box)
        self.signals.create_box_signal.connect(self.model.add_box)
        self.current_dir = None
        self.actions = {}
        self.init_actions()
        self.box_saver = BBoxList()
        self.need_labels = False
        self.undo_redo_manager = UndoRedoManager()
        self.backup_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "backup")

    @classmethod
    def register_action(cls, action_name, action_class):
        if not hasattr(cls, '_action_classes'):
            cls._action_classes = {}
        cls._action_classes[action_name] = action_class

    def init_actions(self):
        if not hasattr(self.__class__, '_action_classes'):
            return
        for name, action_class in self.__class__._action_classes.items():
            self.actions[name] = action_class()

    def set_current_dir(self, dir_path):
        if os.path.isdir(dir_path):
            self.current_dir = dir_path
            self.box_saver = BBoxList(dir_path, "bbox_output.csv")
            return True
        return False

    def get_path(self, im_name):
        if self.current_dir is None:
            return ""
        path = os.path.join(self.current_dir, im_name)
        return path

    def cleanup(self):
        self.model.set_data([], None)
        self.current_dir = None
        self.signals.all_images_deleted_signal.emit()

    def do_action(self, action_name, *args):
        if action_name in self.actions:
            action = self.actions[action_name]
            command = Command(action, *args)
            self.undo_redo_manager.execute(command)

    def clear_backup(self):
        import shutil
        if os.path.exists(self.backup_dir):
            shutil.rmtree(self.backup_dir)