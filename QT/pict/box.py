from PyQt6 import QtWidgets, QtGui, QtCore

class Box(QtWidgets.QGraphicsRectItem):
    def __init__(self, rect=None, label="", image_path="", parent=None):
        rect = rect if rect else QtCore.QRectF()
        super().__init__(rect, parent)
        self.label = label
        self.image_path = image_path
        self.name = image_path.split("\\")[-1].split(".")[0]

        self.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.yellow, 2, QtCore.Qt.PenStyle.SolidLine))
        self.setFlag(QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)

    def update_label(self, new_label):
        self.label = new_label
    
    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.RightButton:
            if self.is_on_border(event.pos()):
                from backend.state import State
                State().actions["DeleteBox"].do(self)
        else:
            super().mousePressEvent(event)

    def is_on_border(self, pos):
        rect = self.rect()
        border_width = 7

        if (rect.left() - border_width <= pos.x() <= rect.right() + border_width and
            rect.top() - border_width <= pos.y() <= rect.bottom() + border_width):

            if (rect.left() < pos.x() < rect.right() and
                rect.top() < pos.y() < rect.bottom()):
                return False 
            return True
        return False