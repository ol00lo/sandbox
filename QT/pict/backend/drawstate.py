from PyQt6 import QtGui, QtCore

class DrawState:
    _instance = None
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._init_state()
        return cls._instance
    
    def _init_state(self):
        self.need_labels = False
        self.line_color = QtGui.QColor(QtCore.Qt.GlobalColor.yellow)
        self.line_width = 4
        self.line_style = QtCore.Qt.PenStyle.SolidLine
        self.margin = 40

        self.label_color = QtGui.QColor(QtCore.Qt.GlobalColor.black)
        self.label_size =  150
        # self.label_size = 12
        self.label_type = "Times New Roman" 
        self.label_offset = 30

    @property
    def pen(self):
        pen = QtGui.QPen(self.line_color, self.line_width, self.line_style)
        pen.setCosmetic(True)
        return pen

    @property
    def label_font(self):
        font = QtGui.QFont(self.label_type)
        font.setPointSize(self.label_size)

        return font

    @property
    def label_pen(self):
        pen = QtGui.QPen(self.label_color, 1, QtCore.Qt.PenStyle.SolidLine)
        pen.setCosmetic(True)
        return pen