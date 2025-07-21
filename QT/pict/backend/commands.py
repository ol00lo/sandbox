class ActionResult:
    def __init__(self, undo_data, redo_data):
        self._undo_data = undo_data
        self._redo_data = redo_data

class Command:
    def __init__(self, action, *args):
        self.action = action
        self.args = args
        self._result_data = None
    
    def execute(self):
        self._result_data = self.action.do(*self.args)
        self._result_data
    
    def undo(self):
        if hasattr(self.action, 'undo') and self._result_data._undo_data:
            return self.action.undo(*self._result_data._undo_data)
    
    def redo(self):
        if hasattr(self.action, 'redo') and self._result_data._redo_data:
            return self.action.redo(*self._result_data._redo_data)


class UndoRedoManager:
    def __init__(self, max_stack=50):
        self._undo_stack = []
        self._redo_stack = []
        self._max_stack = max_stack
    
    def execute(self, command):
        command.execute()
        self._undo_stack.append(command)
        if len(self._undo_stack) > self._max_stack:
            self._undo_stack.pop(0)
        self._redo_stack.clear()
    
    def undo(self):
        if self._undo_stack:
            command = self._undo_stack.pop()
            command.undo()
            self._redo_stack.append(command)
    
    def redo(self):
        if self._redo_stack:
            command = self._redo_stack.pop()
            command.redo()
            self._undo_stack.append(command)

    def is_undo_available(self):
        return bool(self._undo_stack)
