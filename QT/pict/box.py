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

        self.resizing = False
        self.resize_edge = None
        self.resize_margin = 7

    def update_label(self, label):
        self.label = label

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.RightButton:
            if self.is_on_border(event.pos()):
                from backend.state import State
                State().actions["DeleteBox"].do(self)
        elif event.button() == QtCore.Qt.MouseButton.LeftButton:
            if self.is_on_border(event.pos()):
                self.start_resizing(event.pos())
            else:
                super().mousePressEvent(event)
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.resizing:
            self.resize_box(event.pos())
        else:
            super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            if self.resizing:
                self.resizing = False
            else:
                super().mouseReleaseEvent(event)

    def is_on_border(self, pos):
        rect = self.rect()
        border_width = self.resize_margin

        if (rect.left() - border_width <= pos.x() <= rect.right() + border_width and
            rect.top() - border_width <= pos.y() <= rect.bottom() + border_width):

            if (rect.left() < pos.x() < rect.right() and
                rect.top() < pos.y() < rect.bottom()):
                return False 
            return True
        return False

    def start_resizing(self, pos):
        self.resizing = True
        self.resize_start_pos = pos
        self.resize_start_rect = self.rect()

    def resize_box(self, pos):
        if not self.resizing:
            return

        dx = pos.x() - self.resize_start_pos.x()
        dy = pos.y() - self.resize_start_pos.y()
        rect = QtCore.QRectF(self.resize_start_rect)

        if self.is_on_border(self.resize_start_pos):
            if abs(pos.x() - rect.left()) < self.resize_margin:
                rect.setLeft(max(0, rect.left() + dx))
            elif abs(pos.x() - rect.right()) < self.resize_margin:
                rect.setRight(min(rect.right() + dx, self.scene().sceneRect().right()))
            if abs(pos.y() - rect.top()) < self.resize_margin:
                rect.setTop(max(0, rect.top() + dy))
            elif abs(pos.y() - rect.bottom()) < self.resize_margin:
                rect.setBottom(min(rect.bottom() + dy, self.scene().sceneRect().bottom()))

        if rect.width() > 5 and rect.height() > 5:
            self.setRect(rect)

    def is_box(self):
        return self.rect().width() >= 5 and self.rect().height() >= 5

