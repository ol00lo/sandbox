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
        self.label = label

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
        self.resize_start_pos = pos
        self.resize_start_rect = self.rect()

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
            self.setRect(rect)
            self.resize_start_rect = rect
            self.resize_start_pos = pos

    def is_box(self):
        return self.rect().width() >= 5 and self.rect().height() >= 5

    def resize_cursors(self, pos):
        left_dist = abs(pos.x() - self.rect().left())
        right_dist = abs(pos.x() - self.rect().right())
        top_dist = abs(pos.y() - self.rect().top())
        bottom_dist = abs(pos.y() - self.rect().bottom())

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
