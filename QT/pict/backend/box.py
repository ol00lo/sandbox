from PyQt6.QtCore import QRectF
from typing import List


class Box(QRectF):
    def __init__(self, label = "", rect = QRectF()):
        super().__init__(rect)
        self.label = label

    def read_and_ret_name(self, row: List[str]):
        if len(row) != 6:
            return
        name, label, x1, y1, x2, y2 = row
        self.setCoords(float(x1), float(y1),float(x2), float(y2))
        self.label = label
        return name

    def write(self, writer, name):
        writer.writerow([
            name, self.label,
            self.left(), self.top(),
            self.right(), self.bottom()
        ])

    def copy_coord(self, box):
        self.setCoords(box.left(), box.top(), box.right(), box.bottom())
