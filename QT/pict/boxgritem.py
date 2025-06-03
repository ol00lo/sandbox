from PyQt6 import QtWidgets, QtGui, QtCore
from backend.box import Box
import os

class BoxGraphicsItem(QtWidgets.QGraphicsRectItem):
    def __init__(self, box:Box = None, image_path="", parent=None):
        rect = box if box else QtCore.QRectF()
        super().__init__(rect, parent)
        self.box = box
        self.image_path = image_path
        self.name = os.path.basename(image_path)

        self.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.yellow, 2, QtCore.Qt.PenStyle.SolidLine))
        self.setFlag(QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)
        self.set_resizing()

    def set_resizing(self):
        self.resize_edge = None
        self.resize_margin = 5

        self.resize_cursor_map = {
            'left': QtCore.Qt.CursorShape.SizeHorCursor,
            'right':  QtCore.Qt.CursorShape.SizeHorCursor,
            'top':  QtCore.Qt.CursorShape.SizeVerCursor,
            'bottom':  QtCore.Qt.CursorShape.SizeVerCursor,
            'top_left':  QtCore.Qt.CursorShape.SizeFDiagCursor,
            'top_right':  QtCore.Qt.CursorShape.SizeBDiagCursor,
            'bottom_left':  QtCore.Qt.CursorShape.SizeBDiagCursor,
            'bottom_right':  QtCore.Qt.CursorShape.SizeFDiagCursor
        }

    def update_label(self, label):
        self.box.label = label

    def update_box(self, rect):
        self.box = rect
        self.setRect(rect)

    def is_on_border(self, pos):
        rect = self.box
        border_width = self.resize_margin

        if (rect.left() - border_width <= pos.x() <= rect.right() + border_width and
            rect.top() - border_width <= pos.y() <= rect.bottom() + border_width):

            if (rect.left() < pos.x() < rect.right() and
                rect.top() < pos.y() < rect.bottom()):
                return False 
            return True
        return False

    def start_resizing(self, pos):
        self.resize_start_pos = pos
        self.resize_start_rect = self.box

    def resize_box(self, pos):
        dx = pos.x() - self.resize_start_pos.x()
        dy = pos.y() - self.resize_start_pos.y()
        rect = QtCore.QRectF(self.resize_start_rect)

        if abs(pos.x() - rect.left()) < self.resize_margin:
            rect.setLeft(max(0, rect.left() + dx))
        elif abs(pos.x() - rect.right()) < self.resize_margin:
            rect.setRight(min(rect.right() + dx, self.scene().sceneRect().right()))
        if abs(pos.y() - rect.top()) < self.resize_margin:
            rect.setTop(max(0, rect.top() + dy))
        elif abs(pos.y() - rect.bottom()) < self.resize_margin:
            rect.setBottom(min(rect.bottom() + dy, self.scene().sceneRect().bottom()))

        if rect.width() > 5 and rect.height() > 5:
            self.update_box(rect)
            self.resize_start_rect = rect
            self.resize_start_pos = pos

    def is_box(self):
        return self.box.width() >= 5 and self.box.height() >= 5

    def resize_cursors(self, pos):
        left_dist = abs(pos.x() - self.box.left())
        right_dist = abs(pos.x() - self.box.right())
        top_dist = abs(pos.y() - self.box.top())
        bottom_dist = abs(pos.y() - self.box.bottom())

        if left_dist < self.resize_margin and top_dist < self.resize_margin: edge = 'top_left'
        elif right_dist < self.resize_margin and top_dist < self.resize_margin: edge = 'top_right'
        elif left_dist < self.resize_margin and bottom_dist < self.resize_margin: edge = 'bottom_left'
        elif right_dist < self.resize_margin and bottom_dist < self.resize_margin: edge = 'bottom_right'
        elif left_dist < self.resize_margin: edge = 'left'
        elif right_dist < self.resize_margin: edge = 'right'
        elif top_dist < self.resize_margin: edge = 'top'
        elif bottom_dist < self.resize_margin: edge = 'bottom'
        else:
            return  QtCore.Qt.CursorShape.ArrowCursor
        return self.resize_cursor_map[edge]
