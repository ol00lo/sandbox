from PyQt6 import QtWidgets, QtGui, QtCore
import cv2
import numpy as np

class ImageModel(QtWidgets.QGraphicsScene):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.current_image_path = ""
        self.base_pixmap_item = None
        self.overlay_pixmap_item = None
        self.base_image = None
        self.blurred_image = None
        self.saved_rects = []
        self.current_rect = None
        self.is_drawing = False

    def display_image(self, image_path):
        self.clear()
        self.current_image_path = image_path
        cv_image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if cv_image is None:
            return

        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.base_image = cv_image

        self.blurred_image = self._process_image(cv_image)

        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QtGui.QImage(cv_image.data, width, height, bytes_per_line, QtGui.QImage.Format.Format_RGB888)
        pixmap = QtGui.QPixmap.fromImage(q_image)

        self.base_pixmap_item = self.addPixmap(pixmap)
        self.setSceneRect(self.base_pixmap_item.boundingRect())

        overlay = QtGui.QPixmap(pixmap.size())
        overlay.fill(QtCore.Qt.GlobalColor.transparent)
        self.overlay_pixmap_item = self.addPixmap(overlay)
        self.overlay_pixmap_item.setZValue(1)

        self.update_mask()

    def _process_image(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
        h, s, v = cv2.split(hsv)

        s = np.clip(s*0.2, 0, 255).astype(np.uint8)
        h = np.mod(h, 180).astype(np.uint8)
        v = np.clip(v*0.5, 0, 255).astype(np.uint8)

        hsv = cv2.merge([h, s, v])
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    def start_drawing(self):
        self.is_drawing = True
        self.update_mask()

    def stop_drawing(self):
        self.is_drawing = False
        self.update_mask()

    def update_mask(self, rect=None):
        if self.base_image is None:
            return

        if not self.is_drawing and not self.saved_rects:
            overlay = QtGui.QPixmap(self.base_pixmap_item.pixmap().size())
            overlay.fill(QtCore.Qt.GlobalColor.transparent)
            self.overlay_pixmap_item.setPixmap(overlay)
            return

        overlay = np.zeros_like(self.base_image)
        overlay[:] = self.blurred_image

        for saved_rect in self.saved_rects:
            x = max(0, min(int(saved_rect.x()), self.base_image.shape[1] - 1))
            y = max(0, min(int(saved_rect.y()), self.base_image.shape[0] - 1))
            w = min(int(saved_rect.width()), self.base_image.shape[1] - x)
            h = min(int(saved_rect.height()), self.base_image.shape[0] - y)

            overlay[y:y+h, x:x+w] = self.base_image[y:y+h, x:x+w]
            cv2.rectangle(overlay, (x, y), (x+w, y+h), (255, 255, 255), 10)

        if self.is_drawing and rect and rect.isValid():
            x = max(0, min(int(rect.x()), self.base_image.shape[1] - 1))
            y = max(0, min(int(rect.y()), self.base_image.shape[0] - 1))
            w = min(int(rect.width()), self.base_image.shape[1] - x)
            h = min(int(rect.height()), self.base_image.shape[0] - y)

            overlay[y:y+h, x:x+w] = self.base_image[y:y+h, x:x+w]
            cv2.rectangle(overlay, (x, y), (x+w, y+h), (255, 255, 255), 10)

        q_image = QtGui.QImage(overlay.data, overlay.shape[1], overlay.shape[0], 
                             3 * overlay.shape[1], QtGui.QImage.Format.Format_RGB888)
        self.overlay_pixmap_item.setPixmap(QtGui.QPixmap.fromImage(q_image))

    def add_saved_rect(self, rect):
        self.saved_rects.append(rect)
        self.update_mask()

    def clear_saved_rects(self):
        self.saved_rects.clear()
        self.update_mask()

    def n_boxes(self):
        return len(self.saved_rects)