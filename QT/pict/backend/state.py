from .table import TableModel
import os
from PyQt6 import QtCore, QtWidgets
from backend.bboxlist import BBoxList
from backend.box import Box

class Signals(QtCore.QObject):
    rename_image_signal = QtCore.pyqtSignal(str)
    all_image_deleted_signal =  QtCore.pyqtSignal()
    load_images_signal = QtCore.pyqtSignal()
    create_box_signal = QtCore.pyqtSignal(str, Box)
    change_boxes_signal = QtCore.pyqtSignal(str)
    delete_box_signal = QtCore.pyqtSignal(str, Box)

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

        self.selected_image = None
        self.current_dir = None
        self.actions = {}
        self.init_actions()
        self.box_saver = BBoxList()

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

    def get_path(self):
        if self.current_dir is None or self.selected_image is None:
            return ""
        path = os.path.join(self.current_dir, self.selected_image)
        return path

    def cleanup(self):
        self.model.set_data([], None)
        self.selected_image = None
