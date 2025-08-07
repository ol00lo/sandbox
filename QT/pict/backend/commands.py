from PyQt6 import QtCore

class ActionResult:
    def __init__(self, undo_data=[], redo_data=[], backup_data=[]):
        self._undo_data = undo_data
        self._redo_data = redo_data
        self._backup_data = backup_data

class Command:
    def __init__(self, action, *args):
        self.action = action
        self.args = args
        self._result_data = None

    def try_execute(self):
        try:
            self._result_data = self.action.do(*self.args)
            return True
        except: return False

    def try_undo(self):
        try:
            if hasattr(self.action, 'undo') and self._result_data._undo_data:
                self.action.undo(*self._result_data._undo_data)
                return True
        except: return False

    def try_redo(self):
        try:
            if hasattr(self.action, 'redo') and self._result_data._redo_data:
                self.action.redo(*self._result_data._redo_data)
                return True
        except: return False

    def get_backup_data(self):
        return self._result_data._backup_data

class UndoRedoManager(QtCore.QObject):
    delete_backup_signal = QtCore.pyqtSignal(list)
    undo_stack_empty_signal = QtCore.pyqtSignal(bool)
    redo_stack_empty_signal = QtCore.pyqtSignal(bool)

    def __init__(self, max_stack=50):
        super().__init__()
        self._undo_stack = []
        self._redo_stack = []
        self._max_stack = max_stack

    def execute(self, command):
        if not command.try_execute(): 
            return

        if len(self._undo_stack) == 1:
            self.undo_stack_empty_signal.emit(False)

        if self._redo_stack:
            self._redo_stack.clear()
            self.redo_stack_empty_signal.emit(True)

        self._undo_stack.append(command)
        if len(self._undo_stack) > self._max_stack:
            backup_data = self._undo_stack[0].get_backup_data()
            self.delete_backup_signal.emit(backup_data)
            self._undo_stack.pop(0)
        self._redo_stack.clear()

    def undo(self):
        if self._undo_stack:
            command = self._undo_stack.pop()
            if not command.try_undo():
                self._undo_stack.append(command)
                return
            backup_data = command.get_backup_data()
            self.delete_backup_signal.emit(backup_data)
            self._redo_stack.append(command)

            if not self._undo_stack:
                self.undo_stack_empty_signal.emit(True)

            if len(self._redo_stack) == 1:
                self.redo_stack_empty_signal.emit(False)

    def redo(self):
        if self._redo_stack:
            command = self._redo_stack.pop()
            if not command.try_redo():
                self._redo_stack.append(command)
                return
            backup_data = command.get_backup_data()
            self.delete_backup_signal.emit(backup_data)
            self._undo_stack.append(command)

            if not self._redo_stack:
                self.redo_stack_empty_signal.emit(True)

            if len(self._undo_stack) == 1:
                self.undo_stack_empty_signal.emit(False)

    def is_undo_available(self):
        return bool(self._undo_stack)

    def clear_stack(self):
        self._undo_stack.clear()
        self._redo_stack.clear()