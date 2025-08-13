from PyQt6 import QtWidgets, QtGui, QtCore
from backend.drawstate import DrawState

class BBoxSettingsDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Bounding Box Settings")
        self.setFixedSize(450, 450)

        self.current_line_color = None
        self.current_label_color = None

        self.setup_ui()
        self.load_current_settings()

    def setup_ui(self):
        self.layout = QtWidgets.QVBoxLayout(self)

        self.setup_bbox_settings()
        self.setup_label_settings()

        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.StandardButton.Ok | 
            QtWidgets.QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        self.layout.addWidget(buttons)

        self.line_width_slider.valueChanged.connect(self.line_width_spin.setValue)
        self.line_width_spin.valueChanged.connect(self.line_width_slider.setValue)

    def setup_bbox_settings(self):
        box_group = QtWidgets.QGroupBox("Bounding Box Settings")
        box_layout = QtWidgets.QGridLayout()

        box_layout.addWidget(QtWidgets.QLabel("Width:"), 0, 0)

        self.line_width_slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.line_width_slider.setRange(1, 10)
        box_layout.addWidget(self.line_width_slider, 0, 1)

        self.line_width_spin = QtWidgets.QSpinBox()
        self.line_width_spin.setRange(1, 10)
        box_layout.addWidget(self.line_width_spin, 0, 2)

        box_layout.addWidget(QtWidgets.QLabel("Color:"), 1, 0)
        self.line_color_button = self.create_color_button(QtCore.Qt.GlobalColor.yellow)
        box_layout.addWidget(self.line_color_button, 1, 1, 1, 2)

        box_layout.addWidget(QtWidgets.QLabel("Style:"), 2, 0)
        self.line_style_combo = QtWidgets.QComboBox()
        self.line_style_combo.addItem("Solid", QtCore.Qt.PenStyle.SolidLine)
        self.line_style_combo.addItem("Dashed", QtCore.Qt.PenStyle.DashLine)
        self.line_style_combo.addItem("Dotted", QtCore.Qt.PenStyle.DotLine)
        self.line_style_combo.addItem("Dash-Dot", QtCore.Qt.PenStyle.DashDotLine)
        box_layout.addWidget(self.line_style_combo, 2, 1, 1, 2)

        box_group.setLayout(box_layout)
        self.layout.addWidget(box_group)

    def setup_label_settings(self):
        label_group = QtWidgets.QGroupBox("Label Settings")
        label_layout = QtWidgets.QGridLayout()

        label_layout.addWidget(QtWidgets.QLabel("Size:"), 0, 0)
        self.font_size_spin = QtWidgets.QSpinBox()
        self.font_size_spin.setRange(6, 36)
        label_layout.addWidget(self.font_size_spin, 0, 1)

        label_layout.addWidget(QtWidgets.QLabel("Color:"), 1, 0)
        self.label_color_button = self.create_color_button(QtCore.Qt.GlobalColor.black)
        label_layout.addWidget(self.label_color_button, 1, 1)

        label_layout.addWidget(QtWidgets.QLabel("Font:"), 2, 0)
        self.font_combo = QtWidgets.QFontComboBox()
        label_layout.addWidget(self.font_combo, 2, 1)

        label_group.setLayout(label_layout)
        self.layout.addWidget(label_group)


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
                self.current_line_color = color
            else:
                self.current_label_color = color

    def load_current_settings(self):
        state = DrawState()
 
        self.line_width_slider.setValue(state.line_width)
        self.line_width_spin.setValue(state.line_width)
        self.update_button_color(self.line_color_button, state.line_color)
        self.current_line_color = state.line_color
        index = self.line_style_combo.findData(state.line_style)
        if index >= 0:
            self.line_style_combo.setCurrentIndex(index)

        self.font_size_spin.setValue(state.label_size)
        self.update_button_color(self.label_color_button, state.label_color)
        self.current_label_color = state.label_color
        self.font_combo.setCurrentFont(QtGui.QFont(state.label_type))

    def get_settings(self):
        return {
            'line_width': self.line_width_spin.value(),
            'line_color': self.current_line_color if self.current_line_color else DrawState().line_color,
            'line_style': self.line_style_combo.currentData(),
            'label_size': self.font_size_spin.value(),
            'label_color': self.current_label_color if self.current_label_color else DrawState().label_color,
            'label_type': self.font_combo.currentFont().family()
        }