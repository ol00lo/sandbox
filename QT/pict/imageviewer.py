from PyQt6 import QtCore, QtWidgets, QtGui
from scene import ImageModel
from backend.bbox import BBoxList

class ImageViewer(QtWidgets.QGraphicsView):
    coordinates_clicked = QtCore.pyqtSignal(int, int)
    box_created = QtCore.pyqtSignal(QtCore.QRect, str, str)

    def __init__(self, parent):
        super().__init__(parent)
        self.setup_ui()
        self.setup_state()

    def setup_ui(self):
        self.image_model = ImageModel(self)
        self.setScene(self.image_model)
        self.setMouseTracking(True)

        self.cross_cursor = QtGui.QCursor(QtCore.Qt.CursorShape.CrossCursor)
        self.default_cursor = QtGui.QCursor()
        self.setCursor(self.default_cursor)

        t = self.image_model.sceneRect().width() // 90
        self.box_pen = QtGui.QPen(QtCore.Qt.GlobalColor.yellow, t, QtCore.Qt.PenStyle.SolidLine)

    def setup_state(self):
        self.drawing = False
        self.start_point = QtCore.QPoint()
        self.current_box = None
        self.current_path = None
        self.box_saver = BBoxList()
        self.box_created.connect(self.handle_new_box)

    def display_image(self, image_path):
        if image_path == self.current_path:
            return

        self.current_path = image_path
        self.image_model.clear_saved_rects()
        self.image_model.display_image(image_path)

        if self.image_model.items():
            self.fitInView(self.image_model.sceneRect(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)

        if hasattr(self, 'box_saver'):
            for bbox in self.box_saver.get_bboxes_for_image(self.current_path):
                self.image_model.add_saved_rect(QtCore.QRectF(bbox[1]))        

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self.start_drawing(event.pos())
        elif event.button() == QtCore.Qt.MouseButton.RightButton:
            self.handle_right_click(event)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if isinstance(self.itemAt(event.pos()), QtWidgets.QGraphicsPixmapItem):
            self.handle_image_hover(event)
        else:
            self.setCursor(self.default_cursor)
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton and self.drawing:
            self.finish_drawing()
        super().mouseReleaseEvent(event)

    def start_drawing(self, pos):
        self.drawing = True
        self.image_model.start_drawing()
        self.start_point = self.mapToScene(pos)

        self.current_box = QtWidgets.QGraphicsRectItem()
        self.current_box.setPen(self.box_pen)
        self.image_model.addItem(self.current_box)
        self.current_rect = QtCore.QRectF(self.start_point, self.start_point)

    def update_drawing(self, pos):
        end_point = self.mapToScene(pos)
        rect = QtCore.QRectF(self.start_point, end_point).normalized()
        self.current_box.setRect(rect)
        self.image_model.update_mask(rect)

    def finish_drawing(self):
        self.drawing = False
        self.image_model.stop_drawing()

        if not self.current_box:
            return

        rect = self.current_box.rect().toRect()
        if rect.width() > 5 and rect.height() > 5:
            self.save_box(rect)

        self.cleanup_drawing()

    def save_box(self, rect):
        label, ok = QtWidgets.QInputDialog.getText(self, "Label", "Enter label:")
        if ok and label:
            self.box_created.emit(rect, label, self.image_model.current_image_path)

    def cleanup_drawing(self):
        self.image_model.removeItem(self.current_box)
        self.current_box = None

    def handle_right_click(self, event):
        click_pos = self.mapToScene(event.pos())
        event.accept()

        for i, rect in reversed(list(enumerate(self.image_model.saved_rects))):
            if rect.contains(click_pos) and self.confirm_delete():
                self.delete_box(i)
                break

    def confirm_delete(self):
        return QtWidgets.QMessageBox.question(
            self, "Delete box", "Do you really want to delete this box?",
            QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No
        ) == QtWidgets.QMessageBox.StandardButton.Yes

    def delete_box(self, index):
        del self.image_model.saved_rects[index]
        self.update_storage()
        self.image_model.update_mask()

    def update_storage(self):
        self.box_saver.clear_bboxes_for_image(self.image_model.current_image_path)
        for rect in self.image_model.saved_rects:
            self.box_saver.add_bbox(rect.toRect(), "", self.image_model.current_image_path)

    def clear_all_boxes(self):
        self.image_model.clear_saved_rects()
        self.box_saver.clear_bboxes_for_image(self.image_model.current_image_path)

    def handle_image_hover(self, event):
        self.setCursor(self.cross_cursor)
        self.emit_coordinates(event.pos())

        if self.drawing and self.current_box:
            self.update_drawing(event.pos())

    def emit_coordinates(self, pos):
        scene_pos = self.mapToScene(pos)
        item = self.itemAt(pos)
        img_pos = item.mapFromScene(scene_pos)
        pixmap = item.pixmap()

        x = max(0, min(int(img_pos.x()), pixmap.width() - 1))
        y = max(0, min(int(img_pos.y()), pixmap.height() - 1))
        self.coordinates_clicked.emit(x, y)

    def handle_new_box(self, rect, label, image_path):
        self.image_model.add_saved_rect(QtCore.QRectF(rect))

    def leaveEvent(self, event):
        self.setCursor(self.default_cursor)
        super().leaveEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.image_model.items():
            self.fitInView(self.image_model.sceneRect(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)
