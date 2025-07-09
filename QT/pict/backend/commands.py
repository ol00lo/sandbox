class Command:
    def __init__(self, action, *args):
        self.action = action
        self.args = args
        self._undo_data = None
        self._redo_data = None
    
    def execute(self):
        result = self.action.do(*self.args)
        if result:
            self._undo_data = result.get('undo_data')
            self._redo_data = result.get('redo_data')
        return result
    
    def undo(self):
        if hasattr(self.action, 'undo') and self._undo_data:
            return self.action.undo(*self._undo_data)
    
    def redo(self):
        if hasattr(self.action, 'redo') and self._redo_data:
            return self.action.redo(*self._redo_data)


class UndoRedoManager:
    def __init__(self, max_stack=50):
        self.undo_stack = []
        self.redo_stack = []
        self.max_stack = max_stack
    
    def execute(self, command):
        command.execute()
        self.undo_stack.append(command)
        if len(self.undo_stack) > self.max_stack:
            self.undo_stack.pop(0)
        self.redo_stack.clear()
    
    def undo(self):
        if self.undo_stack:
            command = self.undo_stack.pop()
            command.undo()
            self.redo_stack.append(command)
    
    def redo(self):
        if self.redo_stack:
            command = self.redo_stack.pop()
            command.redo()
            self.undo_stack.append(command)