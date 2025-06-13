from PyQt6 import QtWidgets, QtGui, QtCore
from backend.box import Box
from backend.state import State
import os

class BoxGraphicsItem(QtWidgets.QGraphicsRectItem):
    def __init__(self, box:Box = None, image_path="", parent=None):
        super().__init__(box if box else QtCore.QRectF(), parent)
        self.original_box = box
        self.temp_box = None
        self.image_path = image_path
        self.name = os.path.basename(image_path)

        self.setPen(QtGui.QPen(QtCore.Qt.GlobalColor.yellow, 2, QtCore.Qt.PenStyle.SolidLine))
        self.setFlag(QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)
        self.set_resizing()
        self.resizing = False

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
        self.original_box.label = label

    def update_box(self, rect):
        self.original_box = rect
        self.setRect(rect)

    def start_resizing(self, pos):
        self.resizing = True
        self.resize_start_pos = pos
        self.temp_box = QtCore.QRectF(self.original_box)
        self.resize_start_rect = QtCore.QRectF(self.original_box)
        self.setRect(self.temp_box)

    def resize_box(self, pos):
        if not self.resizing or not self.temp_box:
            return

        dx = pos.x() - self.resize_start_pos.x()
        dy = pos.y() - self.resize_start_pos.y()
        rect = QtCore.QRectF(self.temp_box)

        if abs(pos.x() - rect.left()) < self.resize_margin:
            rect.setLeft(max(0, rect.left() + dx))
        elif abs(pos.x() - rect.right()) < self.resize_margin:
            rect.setRight(min(rect.right() + dx, self.scene().sceneRect().right()))
        if abs(pos.y() - rect.top()) < self.resize_margin:
            rect.setTop(max(0, rect.top() + dy))
        elif abs(pos.y() - rect.bottom()) < self.resize_margin:
            rect.setBottom(min(rect.bottom() + dy, self.scene().sceneRect().bottom()))

        if rect.width() > 5 and rect.height() > 5:
            self.temp_box = rect
            self.setRect(rect)
            self.resize_start_pos = pos

    def end_resizing(self):
        if not self.resizing or not self.temp_box:
            return

        old_box = self.original_box
        new_box = QtCore.QRectF(self.temp_box)
        if new_box.width() >= 5 and new_box.height() >= 5:
            self.original_box = new_box
            self.resizing = False
            self.temp_box = None
            self.setRect(self.original_box)
            State().actions["ResizeBox"].do(old_box, new_box, self.image_path)


    def is_box(self):
        return self.original_box.width() >= 5 and self.original_box.height() >= 5

    def resize_cursors(self, pos):
        left_dist = abs(pos.x() - self.original_box.left())
        right_dist = abs(pos.x() - self.original_box.right())
        top_dist = abs(pos.y() - self.original_box.top())
        bottom_dist = abs(pos.y() - self.original_box.bottom())

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

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self.start_resizing(event.pos())
        if event.button() == QtCore.Qt.MouseButton.RightButton:
            question = QtWidgets.QMessageBox()
            question.setIcon(QtWidgets.QMessageBox.Icon.Question)
            question.setWindowTitle('Delete All Images')
            question.setText('Are you sure you want to delete all images?')
            question.setStandardButtons(
                 QtWidgets.QMessageBox.StandardButton.Yes | 
                 QtWidgets.QMessageBox.StandardButton.No
            )
            question.setDefaultButton(QtWidgets.QMessageBox.StandardButton.No)
            reply = question.exec()

            if reply == QtWidgets.QMessageBox.StandardButton.Yes:
                State().actions["DeleteBox"].do(self.rect(), self.image_path)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.resizing:
            self.resize_box(event.pos())
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self.end_resizing()
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def contains(self, point: QtCore.QPointF) -> bool:
        rect = self.rect()
        margin = self.resize_margin

        outer_rect = rect.adjusted(-margin, -margin, margin, margin)

        inner_rect = rect.adjusted(margin, margin, -margin, -margin)

        return outer_rect.contains(point) and not inner_rect.contains(point)

    def shape(self) -> QtGui.QPainterPath:
        path = QtGui.QPainterPath()
        rect = self.rect()
        margin = self.resize_margin

        outer_rect = rect.adjusted(-margin, -margin, margin, margin)
        path.addRect(outer_rect)

        inner_rect = rect.adjusted(margin, margin, -margin, -margin)
        inner_path = QtGui.QPainterPath()
        inner_path.addRect(inner_rect)
        path = path.subtracted(inner_path)

        return path