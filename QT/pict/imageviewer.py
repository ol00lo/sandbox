from PyQt6 import QtCore, QtWidgets, QtGui
from scene import ImageModel
from backend.bbox import BBoxList

class ImageViewer(QtWidgets.QGraphicsView):
    coordinates_clicked = QtCore.pyqtSignal(int, int)
    box_created = QtCore.pyqtSignal(QtCore.QRect, str, str)

    def __init__(self, parent):
        super().__init__(parent)
        self.parent = parent

        self.image_model = ImageModel(self)
        self.setScene(self.image_model)
        self.setMouseTracking(True)

        self.cross_cursor = QtGui.QCursor(QtCore.Qt.CursorShape.CrossCursor)
        self.default_cursor = QtGui.QCursor()
        self.setCursor(self.default_cursor)

        self.drawing = False
        self.start_point = QtCore.QPoint()
        self.current_box = None
        self.current_path = None
        t = self.image_model.sceneRect().width() // 90
        self.box_pen = QtGui.QPen(QtCore.Qt.GlobalColor.yellow, t, QtCore.Qt.PenStyle.SolidLine)
        self.box_saver = BBoxList()
        self.box_created.connect(self.box_saver.add_bbox)
        self.box_created.connect(self.on_box_created)

    def on_box_created(self, rect, label, image_path):
        self.image_model.add_saved_rect(QtCore.QRectF(rect))

    def display_image(self, image_path):
        if image_path == self.current_path:
            return
        self.image_model.clear_saved_rects()
        self.current_path = image_path
        self.image_model.display_image(image_path)
        self.fitInView(self.image_model.sceneRect(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)
        if hasattr(self, 'box_saver'):
            for bbox in self.box_saver.get_bboxes_for_image(image_path):
                self.image_model.add_saved_rect(QtCore.QRectF(bbox[1]))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.image_model.items():
            self.fitInView(self.image_model.sceneRect(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)

    def mouseMoveEvent(self, event):
        item = self.itemAt(event.pos())
        if isinstance(item, QtWidgets.QGraphicsPixmapItem):
            self.setCursor(self.cross_cursor)

            scene_pos = self.mapToScene(event.pos())
            img_pos = item.mapFromScene(scene_pos)
            pixmap = item.pixmap()

            x = max(0, min(int(img_pos.x()), pixmap.width() - 1))
            y = max(0, min(int(img_pos.y()), pixmap.height() - 1))

            self.coordinates_clicked.emit(x, y)

            if self.drawing and self.current_box:
                end_point = self.mapToScene(event.pos())
                rect = QtCore.QRectF(self.start_point, end_point).normalized()
                self.current_box.setRect(rect)
                self.image_model.update_mask(rect)
        else:
            self.setCursor(self.default_cursor)
        super().mouseMoveEvent(event)

    def leaveEvent(self, event):
        self.setCursor(self.default_cursor)
        super().leaveEvent(event)

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            self.drawing = True
            self.image_model.start_drawing()
            self.start_point = self.mapToScene(event.pos())
            self.current_box = QtWidgets.QGraphicsRectItem()
            self.current_box.setPen(self.box_pen)
            self.image_model.addItem(self.current_box)
            self.current_rect = QtCore.QRectF(self.start_point, self.start_point)
        elif event.button() == QtCore.Qt.MouseButton.RightButton:
            event.accept()
            self.handle_right_click(event)
        super().mousePressEvent(event)

    def handle_right_click(self, event):
        click_pos = self.mapToScene(event.pos())

        for i in reversed(range(len(self.image_model.saved_rects))):
            rect = self.image_model.saved_rects[i]

            if rect.contains(click_pos):
                answer = QtWidgets.QMessageBox.question(
                    self,
                    "Delete box",
                    "Do you really want to delete this box?",
                    QtWidgets.QMessageBox.StandardButton.Yes | QtWidgets.QMessageBox.StandardButton.No
                )

                if answer == QtWidgets.QMessageBox.StandardButton.Yes:
                    del self.image_model.saved_rects[i]

                    self.box_saver.clear_bboxes_for_image(self.image_model.current_image_path)
                    for rect in self.image_model.saved_rects:
                        self.box_saver.add_bbox(rect.toRect(), "", self.image_model.current_image_path)

                    self.image_model.update_mask()
                break

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.MouseButton.LeftButton and self.drawing:
            self.drawing = False
            self.image_model.stop_drawing()

            if self.current_box:
                rect = self.current_box.rect().toRect()
                if rect.width() > 5 and rect.height() > 5:
                    label, ok = QtWidgets.QInputDialog.getText(self, "Label", "Enter label:")
                    if ok and label:
                        self.box_created.emit(rect, label, self.image_model.current_image_path)

                self.image_model.removeItem(self.current_box)
                self.current_box = None
        super().mouseReleaseEvent(event)

    def clear_all_boxes(self):
        self.image_model.clear_saved_rects()
        self.box_saver.clear_bboxes_for_image(self.image_model.current_image_path)
