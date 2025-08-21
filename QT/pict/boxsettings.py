from PyQt6 import QtWidgets, QtGui, QtCore
from backend.drawstate import DrawState

class BBoxSettingsDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Bounding Box Settings")
        self.setFixedSize(450, 500)

        self.line_width = QtWidgets.QSpinBox()
        self.line_color_button = QtWidgets.QPushButton()
        self.line_color = None
        self.line_style = QtWidgets.QComboBox()

        self.font_size = QtWidgets.QSpinBox()
        self.label_color_button = QtWidgets.QPushButton()
        self.label_color = None
        self.font = QtWidgets.QFontComboBox()
        self.bg_alpha = QtWidgets.QSpinBox()

        self.layout = QtWidgets.QVBoxLayout(self)

        self.load_current_settings()
        self.setup_bbox_settings()
        self.setup_label_settings()
        self.setup_preview()
        self.setup_acept_reject()

    def setup_bbox_settings(self):
        box_group = QtWidgets.QGroupBox("Bounding Box Settings")
        box_layout = QtWidgets.QGridLayout()
        box_layout.setColumnMinimumWidth(1, 200)

        box_layout.addWidget(QtWidgets.QLabel("Width:"), 0, 0)
        self.line_width.setRange(1, 10)
        box_layout.addWidget(self.line_width, 0, 1)

        box_layout.addWidget(QtWidgets.QLabel("Color:"), 1, 0)
        self.line_color_button = self.create_color_button(DrawState().line_color)
        box_layout.addWidget(self.line_color_button, 1, 1)

        box_layout.addWidget(QtWidgets.QLabel("Style:"), 2, 0)
        self.line_style.addItem("Solid", QtCore.Qt.PenStyle.SolidLine)
        self.line_style.addItem("Dashed", QtCore.Qt.PenStyle.DashLine)
        self.line_style.addItem("Dotted", QtCore.Qt.PenStyle.DotLine)
        self.line_style.addItem("Dash-Dot", QtCore.Qt.PenStyle.DashDotLine)
        box_layout.addWidget(self.line_style, 2, 1)

        box_group.setLayout(box_layout)
        self.layout.addWidget(box_group)

    def setup_label_settings(self):
        label_group = QtWidgets.QGroupBox("Label Settings")
        label_layout = QtWidgets.QGridLayout()
        label_layout.setColumnMinimumWidth(1, 200)

        label_layout.addWidget(QtWidgets.QLabel("Size:"), 0, 0)
        self.font_size.setRange(6, 36)
        label_layout.addWidget(self.font_size, 0, 1)

        label_layout.addWidget(QtWidgets.QLabel("Color:"), 1, 0)
        self.label_color_button = self.create_color_button(DrawState().label_color)
        label_layout.addWidget(self.label_color_button, 1, 1)

        label_layout.addWidget(QtWidgets.QLabel("Font:"), 2, 0)
        label_layout.addWidget(self.font, 2, 1)

        label_layout.addWidget(QtWidgets.QLabel("Background opacity:"), 3, 0)
        self.bg_alpha.setRange(0, 255)
        label_layout.addWidget(self.bg_alpha, 3, 1)

        label_group.setLayout(label_layout)
        self.layout.addWidget(label_group)

    def setup_preview(self):
        preview_group = QtWidgets.QGroupBox("Preview")
        preview_layout = QtWidgets.QHBoxLayout()

        self.bbox_preview = QtWidgets.QGraphicsView()
        self.bbox_preview_scene = QtWidgets.QGraphicsScene()
        self.bbox_preview.setScene(self.bbox_preview_scene)

        self.preview_rect = QtWidgets.QGraphicsRectItem(0, 0, 160, 80)
        self.bbox_preview_scene.addItem(self.preview_rect)

        self.preview_text = QtWidgets.QGraphicsSimpleTextItem("Label")
        self.bbox_preview_scene.addItem(self.preview_text)
        self.preview_text_bg = QtWidgets.QGraphicsRectItem(self.preview_text)
        self.preview_text_bg.setPen(QtGui.QPen(QtCore.Qt.PenStyle.NoPen))
        self.preview_text_bg.setFlag(QtWidgets.QGraphicsItem.GraphicsItemFlag.ItemStacksBehindParent)

        self.update_bbox_preview()
        self.update_text_preview()
        self.update_text_background_brush()
        preview_layout.addWidget(self.bbox_preview)

        preview_group.setLayout(preview_layout)
        self.layout.addWidget(preview_group)

        self.line_width.valueChanged.connect(self.update_bbox_preview)
        self.line_color_button.clicked.connect(self.update_bbox_preview)
        self.line_style.currentIndexChanged.connect(self.update_bbox_preview)
        self.font_size.valueChanged.connect(self.update_text_preview)
        self.label_color_button.clicked.connect(self.update_text_preview)
        self.font.currentFontChanged.connect(self.update_text_preview)
        self.bg_alpha.valueChanged.connect(self.update_text_preview)
        self.bg_alpha.valueChanged.connect(self.update_text_background_brush)

    def update_bbox_preview(self):
        pen = QtGui.QPen()
        pen.setWidth(self.line_width.value())
        pen.setColor(self.line_color)
        pen.setStyle(self.line_style.currentData())
        self.preview_rect.setPen(pen)
        self.update_text_background_brush()
        self.update_text_position()

    def update_text_preview(self):
        font = QtGui.QFont(self.font.currentFont())
        font.setPointSize(self.font_size.value())
        self.preview_text.setFont(font)
        self.preview_text.setBrush(QtGui.QBrush(self.label_color))
        self.update_text_background_brush()
        self.update_text_position()

    def update_text_background_brush(self):
        if hasattr(self, 'preview_text_bg'):
            bg = QtGui.QColor(self.line_color)
            bg.setAlpha(self.bg_alpha.value())
            self.preview_text_bg.setBrush(QtGui.QBrush(bg))

    def update_text_position(self):
        if hasattr(self, 'preview_text') and hasattr(self, 'preview_rect'):
            rect = self.preview_rect.rect()
            padding = 2.0
            # Position top-left above the box
            text_x = rect.left() + padding
            text_y = rect.top() - self.preview_text.boundingRect().height() - padding
            self.preview_text.setPos(text_x, text_y)

            # Update background geometry with padding
            if hasattr(self, 'preview_text_bg'):
                text_rect_local = self.preview_text.boundingRect()
                self.preview_text_bg.setRect(
                    text_rect_local.adjusted(-padding, -padding, padding, padding)
                )

    def create_color_button(self, default_color):
        button = QtWidgets.QPushButton()
        button.setFixedSize(30, 30)
        self.update_button_color(button, default_color)
        button.clicked.connect(lambda: self.choose_color(button))
        return button

    def update_button_color(self, button, color):
        pixmap = QtGui.QPixmap(24, 24)
        pixmap.fill(color)
        button.setIcon(QtGui.QIcon(pixmap))
        button.setIconSize(QtCore.QSize(22, 22))

    def choose_color(self, button):
        current_color = button.icon().pixmap(24, 24).toImage().pixelColor(0, 0)
        color = QtWidgets.QColorDialog.getColor(current_color, self)
        if color.isValid():
            self.update_button_color(button, color)
            if button == self.line_color_button:
                self.line_color = color
            else:
                self.label_color = color

    def load_current_settings(self):
        self.line_width.setValue(DrawState().line_width)
        self.line_color = DrawState().line_color

        index = self.line_style.findData(DrawState().line_style)
        if index >= 0:
            self.line_style.setCurrentIndex(index)

        self.font_size.setValue(DrawState().label_size)
        self.label_color = DrawState().label_color
        self.font.setCurrentFont(QtGui.QFont(DrawState().label_type))
        self.bg_alpha.setValue(DrawState().label_background_alpha)

    def setup_acept_reject(self):
        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.StandardButton.Ok | 
            QtWidgets.QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        self.layout.addWidget(buttons)

    def get_settings(self):
        return {
            'line_width': self.line_width.value(),
            'line_color': self.line_color,
            'line_style': self.line_style.currentData(),
            'label_size': self.font_size.value(),
            'label_color': self.label_color,
            'label_type': self.font.currentFont().family(),
            'label_background_alpha': self.bg_alpha.value()
        }