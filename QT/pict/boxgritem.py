from PyQt6 import QtWidgets, QtGui, QtCore
from backend.box import Box
from backend.state import State
import os
from backend.drawstate import DrawState

class BoxGraphicsItem(QtWidgets.QGraphicsRectItem):
    def __init__(self, box:Box = None, image_path="", parent=None):
        super().__init__(box if box else QtCore.QRectF(), parent)
        self.original_box = box
        self.temp_box = None
        self.image_path = image_path
        self.name = os.path.basename(image_path)
        self.label_item = None

        self.setAcceptHoverEvents(True)

        self.setPen(DrawState().pen)

        self.setFlag(QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemIsSelectable)
        self.set_resizing()

    def set_resizing(self):
        self.resizing = False
        self.resize_edge = None

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
        State().signals.update_mask_signal.emit(rect)

    def start_resizing(self, pos):
        self.resizing = True
        self.resize_start_pos = pos
        self.temp_box = QtCore.QRectF(self.original_box)
        self.resize_start_rect = QtCore.QRectF(self.original_box)

        self.h_resize = None
        self.v_resize = None
        view = self.scene().views()[0]
        margin = DrawState().margin / view.transform().m11()

        if abs(pos.x() - self.rect().left()) < margin:
            self.h_resize = 'left'
        elif abs(pos.x() - self.rect().right()) < margin:
            self.h_resize = 'right'

        if abs(pos.y() - self.rect().top()) < margin:
            self.v_resize = 'top'
        elif abs(pos.y() -self.rect().bottom()) < margin:
            self.v_resize = 'bottom'

        self.is_corner_resize = (self.h_resize is not None and self.v_resize is not None)

        self.setRect(self.temp_box)
        State().signals.create_mask_signal.emit(self.temp_box)

    def resize_box(self, pos):
        if not self.resizing or not self.temp_box:
            return

        dx = pos.x() - self.resize_start_pos.x()
        dy = pos.y() - self.resize_start_pos.y()
        rect = QtCore.QRectF(self.temp_box)
        scene_rect = self.scene().sceneRect()

        if self.h_resize == 'left':
            new_left = rect.left() + dx
            rect.setLeft(max(0, new_left))
        elif self.h_resize == 'right':
            new_right = rect.right() + dx
            rect.setRight(min(new_right, scene_rect.right()))

        if self.v_resize == 'top':
            new_top = rect.top() + dy
            rect.setTop(max(0, new_top))
        elif self.v_resize == 'bottom':
            new_bottom = rect.bottom() + dy
            rect.setBottom(min(new_bottom, scene_rect.bottom()))

        if rect.width() > 5 and rect.height() > 5:
            self.temp_box = rect
            self.setRect(rect)
            State().signals.update_mask_signal.emit(rect)
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
            State().signals.delete_mask_signal.emit()
            State().do_action("ResizeBox", old_box, new_box, self.image_path)

    def is_box(self):
        return self.original_box.width() >= 5 and self.original_box.height() >= 5

    def resize_cursors(self, pos):
        left_dist = abs(pos.x() - self.original_box.left())
        right_dist = abs(pos.x() - self.original_box.right())
        top_dist = abs(pos.y() - self.original_box.top())
        bottom_dist = abs(pos.y() - self.original_box.bottom())

        view = self.scene().views()[0]
        margin = DrawState().margin / view.transform().m11()

        if left_dist < margin and top_dist < margin: edge = 'top_left'
        elif right_dist < margin and top_dist < margin: edge = 'top_right'
        elif left_dist < margin and bottom_dist < margin: edge = 'bottom_left'
        elif right_dist < margin and bottom_dist < margin: edge = 'bottom_right'
        elif left_dist < margin: edge = 'left'
        elif right_dist < margin: edge = 'right'
        elif top_dist < margin: edge = 'top'
        elif bottom_dist < margin: edge = 'bottom'
        else:
            return  QtCore.Qt.CursorShape.ArrowCursor
        return self.resize_cursor_map[edge]

    def mousePressEvent(self, event):
        try:
            if event.button() == QtCore.Qt.MouseButton.LeftButton:
                self.start_resizing(event.pos())
                event.accept()
                return

            if event.button() == QtCore.Qt.MouseButton.RightButton:
                question = QtWidgets.QMessageBox()
                question.setIcon(QtWidgets.QMessageBox.Icon.Question)
                question.setWindowTitle('Delete Box')
                question.setText('Are you sure you want to delete this box?')
                question.setStandardButtons(
                     QtWidgets.QMessageBox.StandardButton.Yes | 
                     QtWidgets.QMessageBox.StandardButton.No
                )
                question.setDefaultButton(QtWidgets.QMessageBox.StandardButton.No)
                reply = question.exec()

                if reply == QtWidgets.QMessageBox.StandardButton.Yes:
                    State().do_action("DeleteBox", self.original_box, self.image_path)
                    event.accept()
                    return
                super().mousePressEvent(event)
        except Exception as e:
            print(e)

    def mouseMoveEvent(self, event):
        try:
            if self.resizing:
                self.resize_box(event.pos())
        except Exception as e:
            print(e)
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        try:
            if event.button() == QtCore.Qt.MouseButton.LeftButton:
                self.end_resizing()
                event.accept()
                return
        except Exception as e:
            print(e)
        super().mouseReleaseEvent(event)

    def hoverEnterEvent(self, event):
        try:
            self.setPen(DrawState().hover_pen)
            if State().need_labels:
                label_item = None
                def callback(item):
                    nonlocal label_item
                    label_item = item
                State().signals.add_label_signal.emit(self.original_box.label, self, True, callback)
                self.label_item = label_item
                self.scene().addItem(self.label_item)
            self.update()
            self.setZValue(100)
            event.accept()
        except Exception as e:
            print(e)

    def hoverLeaveEvent(self, event):
        try:
            self.setPen(DrawState().pen)
            if State().need_labels and self.label_item:
                self.scene().removeItem(self.label_item)
                self.label_item = None
            self.update()
            self.setZValue(2)
            event.accept()
        except Exception as e:
            print(e)

    def contains(self, point: QtCore.QPointF) -> bool:
        rect = self.rect()
        view = self.scene().views()[0]
        margin = DrawState().margin / view.transform().m11()

        outer_rect = rect.adjusted(-margin, -margin, margin, margin)

        inner_rect = rect.adjusted(margin, margin, -margin, -margin)

        return outer_rect.contains(point) and not inner_rect.contains(point)

    def shape(self) -> QtGui.QPainterPath:
        path = QtGui.QPainterPath()
        rect = self.rect()
        view = self.scene().views()[0]
        margin = DrawState().margin / view.transform().m11()

        outer_rect = rect.adjusted(-margin, -margin, margin, margin)
        path.addRect(outer_rect)

        inner_rect = rect.adjusted(margin, margin, -margin, -margin)
        inner_path = QtGui.QPainterPath()
        inner_path.addRect(inner_rect)
        path = path.subtracted(inner_path)

        return path