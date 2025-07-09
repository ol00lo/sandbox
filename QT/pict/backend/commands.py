class Command:
    def __init__(self, action, *args):
        self.action = action
        self.args = args
        self._saved_state = None 
        self._redo_state = None
    
    def execute(self):
        self._saved_state = self.action.do(*self.args)
        return self._saved_state
    
    def undo(self):
        self._redo_state = self.action.undo(*self._saved_state)
    
    def redo(self):
        self.action.redo(*self._redo_state)


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