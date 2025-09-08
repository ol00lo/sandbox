from PyQt6 import QtCore, QtWidgets, QtGui
from scene import ImageModel
from boxgritem import BoxGraphicsItem
from backend.state import State
from backend.box import Box
from qt_common import show_message
from drawstate import DrawState

class ImageViewer(QtWidgets.QGraphicsView):
    coordinates_clicked = QtCore.pyqtSignal(int, int)

    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent

        self.image_model = ImageModel(self)
        self.setScene(self.image_model)
        self.setMouseTracking(True)
        self.set_f()

    def set_f(self):
        self.cross_cursor = QtGui.QCursor(QtCore.Qt.CursorShape.CrossCursor)
        self.default_cursor = QtGui.QCursor()
        self.setCursor(self.default_cursor)

        self.drawing = False
        self.start_point = QtCore.QPoint()
        self.current_box: Box = None

    def display_image(self):
        self.image_model.display_image()
        self.fitInView(self.image_model.sceneRect(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)
        self.image_model.update_boxes()

    def update_image(self):
        self.image_model.update_boxes()

    def mousePressEvent(self, event):
        super().mousePressEvent(event)
        try:
            if event.isAccepted():
                return
            items_under_cursor = self.scene().items(self.mapToScene(event.pos()))
            is_picture = isinstance(items_under_cursor[0], QtWidgets.QGraphicsPixmapItem) if items_under_cursor else False
            if event.button() == QtCore.Qt.MouseButton.LeftButton and is_picture:
                self.start_drawing(event)
                event.accept()
                return
        except Exception as e:
            show_message(message=str(e), is_error=True)

    def mouseMoveEvent(self, event):
        try:
            pos = event.pos()
            scene_pos = self.mapToScene(pos)
            item = self.itemAt(event.pos())
            if isinstance(item, QtWidgets.QGraphicsPixmapItem):
                self.setCursor(self.cross_cursor)
                self.track_coordinates(event)
            if not self.drawing:
                self.set_cursor(scene_pos, item)
            if self.drawing and self.current_box:
                self.update_current_box(event)
        except Exception as e:
            show_message(message=str(e), is_error=True)
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        try:
            if event.button() == QtCore.Qt.MouseButton.LeftButton and self.drawing:
                self.finish_drawing()
        except Exception as e:
            show_message(message=str(e), is_error=True)
        super().mouseReleaseEvent(event)

    def track_coordinates(self, event):
        pos = event.pos()
        scene_pos = self.mapToScene(pos)
        item = self.itemAt(pos)
        img_pos = item.mapFromScene(scene_pos)
        pixmap = item.pixmap()

        x = max(0, min(int(img_pos.x()), pixmap.width() - 1))
        y = max(0, min(int(img_pos.y()), pixmap.height() - 1))

        self.coordinates_clicked.emit(x, y)

    def update_current_box(self, event):
        end_point = self.mapToScene(event.pos())
        scene_rect = self.image_model.sceneRect()

        x = max(scene_rect.left(), min(end_point.x(), scene_rect.right()))
        y = max(scene_rect.top(), min(end_point.y(), scene_rect.bottom()))
        clamped_end_point = QtCore.QPointF(x, y)

        self.current_box.update_box(QtCore.QRectF(self.start_point, clamped_end_point).normalized())

    def start_drawing(self, event):
        self.drawing = True
        self.start_point = self.mapToScene(event.pos())

        scene_rect = Box("", QtCore.QRectF(self.start_point, self.start_point))
        self.current_box = BoxGraphicsItem(box = scene_rect, image_path=self.image_model.current_image_path)
        DrawState().signals.create_mask_signal.emit(scene_rect)
        self.image_model.addItem(self.current_box)

    def finish_drawing(self):
        self.drawing = False

        if not self.current_box or not self.current_box.is_box():
            self.cancel_drawing()
            return

        dialog = QtWidgets.QInputDialog(self)
        dialog.setInputMode(QtWidgets.QInputDialog.InputMode.TextInput)
        dialog.setLabelText("Enter label:")
        line_edit = dialog.findChild(QtWidgets.QLineEdit)
        if line_edit:
            validator = QtGui.QRegularExpressionValidator(QtCore.QRegularExpression("[a-zA-Z0-9]*"), self)
            line_edit.setValidator(validator)
        dialog.accepted.connect(lambda: self.end_drawing(dialog.textValue()))
        dialog.rejected.connect(self.cancel_drawing)
        dialog.open()

    def end_drawing(self, label):
        if label == "":
            self.cancel_drawing()
            return

        self.current_box.update_label(label)
        State().do_action("CreateBox", self.current_box.original_box, self.current_box.name)
        if State().need_labels:
            self.image_model.add_labels(self.current_box, label)
        self.current_box = None
        DrawState().signals.delete_mask_signal.emit()

    def cancel_drawing(self):
        self.image_model.removeItem(self.current_box)
        self.current_box = None
        DrawState().signals.delete_mask_signal.emit()

    def set_cursor(self, pos, item):
        if isinstance(item, QtWidgets.QGraphicsPixmapItem):
            self.setCursor(self.cross_cursor)
        elif isinstance(item, BoxGraphicsItem) and item.contains(pos):
            if item.resizing:
                self.setCursor(self.default_cursor)
            else:
                self.setCursor(item.resize_cursors(pos))
        else:
            self.setCursor(self.default_cursor)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        try:
            self.fitInView(self.image_model.sceneRect(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)
            self.image_model.update_boxes()
        except Exception as e:
            show_message(message=str(e), is_error=True)
