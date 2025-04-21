from .table import TableModel
import os
from PyQt6 import QtCore, QtWidgets

class Signals(QtCore.QObject):
    image_selected = QtCore.pyqtSignal(str)
    load_images_signal = QtCore.pyqtSignal()
    curr_dir_signal = QtCore.pyqtSignal(str)

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

        self.current_dir = None
        self.actions = {}
        self.init_actions()

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
            return True
        return False

    def get_path(self):
        if self.current_dir is None or self.selected_image is None:
            return ""
        path = os.path.join(self.current_dir, self.selected_image)
        return path

    def cleanup(self):
        self.model.set_data([], None)
        self.current_dir = None
        self.selected_image = None
