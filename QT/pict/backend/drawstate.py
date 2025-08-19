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

        r, g, b = 255, 215, 0

        self.line_color = QtGui.QColor(r, g, b)
        self.line_width = 4
        self.line_style = QtCore.Qt.PenStyle.SolidLine
        self.margin = 10

        self.how_hover_brighter = 150
        self.how_hover_wider = 1
        self.hover_line_style = QtCore.Qt.PenStyle.SolidLine

        self.label_color = QtGui.QColor(QtCore.Qt.GlobalColor.black)
        #self.label_size =  150
        self.label_size = 12
        self.label_type = "Times New Roman" 

        self.dark_mask_color = QtGui.QColor(0, 0, 0, 150)

    @property
    def pen(self):
        pen = QtGui.QPen(self.line_color, self.line_width, self.line_style)
        pen.setCosmetic(True)
        return pen

    @property
    def hover_pen(self):
        pen = QtGui.QPen(self.hover_line_color, self.hover_line_width, self.hover_line_style)
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

    @property
    def hover_line_color(self):
        return self.line_color.lighter(self.how_hover_brighter)

    @property
    def hover_line_width(self):
        return self.line_width + self.how_hover_wider
