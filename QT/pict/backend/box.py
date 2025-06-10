from PyQt6.QtCore import QRectF

class Box(QRectF):
    def __init__(self, label, rect):
        super().__init__(rect)
        self.label = label