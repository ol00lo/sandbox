from PyQt6 import QtCore, QtWidgets, QtGui
from PyQt6.QtCore import Qt
from scene import ImageModel
from backend.bbox import BBoxList

class ImageViewer(QtWidgets.QGraphicsView):
    coordinates_clicked = QtCore.pyqtSignal(int, int)
    box_created = QtCore.pyqtSignal(QtCore.QRect, str, str)

    def __init__(self, parent):
        super().__init__(parent)
        self.setup_ui()
        self.setup_state()
        self.setup_resize_handles()

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

    def setup_resize_handles(self):
        self.selected_box_index = -1
        self.resizing = False
        self.resize_edge = None
        self.resize_margin = 10

        self.resize_cursors = {
            'left': Qt.CursorShape.SizeHorCursor,
            'right': Qt.CursorShape.SizeHorCursor,
            'top': Qt.CursorShape.SizeVerCursor,
            'bottom': Qt.CursorShape.SizeVerCursor,
            'top_left': Qt.CursorShape.SizeFDiagCursor,
            'top_right': Qt.CursorShape.SizeBDiagCursor,
            'bottom_left': Qt.CursorShape.SizeBDiagCursor,
            'bottom_right': Qt.CursorShape.SizeFDiagCursor
        }

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

    # ========== EVENTS ==========
    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            scene_pos = self.mapToScene(event.pos())
            if self.resize_edge and self.selected_box_index >= 0:
                self.start_resizing(scene_pos)
                event.accept()
                return
            else:
                self.start_drawing(event.pos())
        elif event.button() == QtCore.Qt.MouseButton.RightButton:
            event.accept()
            self.handle_right_click(event)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        pos = event.pos()
        scene_pos = self.mapToScene(pos)
        if not self.drawing and not self.resizing:
            self.update_resize_cursor(scene_pos)
        if self.drawing and self.current_box:
            self.update_drawing(pos)
        elif self.resizing and self.selected_box_index >= 0:
            self.resize_box(scene_pos)
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            if self.resizing:
                self.resizing = False
                self.update_storage()
                event.accept()
                return
            elif self.drawing:
                self.finish_drawing()
        super().mouseReleaseEvent(event)
    
    def leaveEvent(self, event):
        self.setCursor(self.default_cursor)
        super().leaveEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.image_model.items():
            self.fitInView(self.image_model.sceneRect(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)

    # ========== RESIZE ==========
    def start_resizing(self, scene_pos):
        self.resizing = True
        self.drawing = False
        self.resize_start_pos = scene_pos
        self.resize_start_rect = QtCore.QRectF(
            self.image_model.saved_rects[self.selected_box_index])

    def resize_box(self, scene_pos):
        if self.selected_box_index < 0 or not self.resize_edge:
            return
        rect = QtCore.QRectF(self.resize_start_rect)
        dx = scene_pos.x() - self.resize_start_pos.x()
        dy = scene_pos.y() - self.resize_start_pos.y()

        if 'left' in self.resize_edge:
            rect.setLeft(max(0, rect.left() + dx))
        if 'right' in self.resize_edge:
            rect.setRight(min(self.image_model.base_image.shape[1], rect.right() + dx))
        if 'top' in self.resize_edge:
            rect.setTop(max(0, rect.top() + dy))
        if 'bottom' in self.resize_edge:
            rect.setBottom(min(self.image_model.base_image.shape[0], rect.bottom() + dy))
        if rect.width() > 5 and rect.height() > 5:
            self.image_model.saved_rects[self.selected_box_index] = rect
            self.image_model.update_mask()

    def update_resize_cursor(self, scene_pos):
        for i, rect in enumerate(self.image_model.saved_rects):
            if rect.contains(scene_pos):
                self.selected_box_index = i

                left_dist = abs(scene_pos.x() - rect.left())
                right_dist = abs(scene_pos.x() - rect.right())
                top_dist = abs(scene_pos.y() - rect.top())
                bottom_dist = abs(scene_pos.y() - rect.bottom())

                if left_dist < self.resize_margin and top_dist < self.resize_margin:
                    edge = 'top_left'
                elif right_dist < self.resize_margin and top_dist < self.resize_margin:
                    edge = 'top_right'
                elif left_dist < self.resize_margin and bottom_dist < self.resize_margin:
                    edge = 'bottom_left'
                elif right_dist < self.resize_margin and bottom_dist < self.resize_margin:
                    edge = 'bottom_right'
                elif left_dist < self.resize_margin:
                    edge = 'left'
                elif right_dist < self.resize_margin:
                    edge = 'right'
                elif top_dist < self.resize_margin:
                    edge = 'top'
                elif bottom_dist < self.resize_margin:
                    edge = 'bottom'
                else:
                    self.setCursor(self.cross_cursor)
                    self.resize_edge = None
                    return
                self.resize_edge = edge
                self.setCursor(QtGui.QCursor(self.resize_cursors[edge]))
                return

        self.selected_box_index = -1
        self.resize_edge = None
        self.setCursor(self.default_cursor)

    # ========== DRAWING ==========
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

    # ========== DELETE ==========
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

    def handle_new_box(self, rect, label, image_path):
        self.image_model.add_saved_rect(QtCore.QRectF(rect))
