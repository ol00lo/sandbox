from table import TableModel
import os

class State:
    _instance = None
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._init_state()
        return cls._instance

    def _init_state(self):
        self.model = TableModel()
        self.current_dir = None
        self.selected_image = None
        self.actions = {
            'delete': None,
            'rename': None
        }

    def register_action(self, name, action):
        self.actions[name] = action

    def get_action(self, name):
        return self.actions.get(name)

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

    def get_model(self):
        return self.model

    def cleanup(self):
        self.model.set_data([], None)
        self.current_dir = None
        self.selected_image = None
