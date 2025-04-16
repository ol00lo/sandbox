from PyQt6 import QtWidgets, QtCore, QtGui

class RightControl(QtWidgets.QWidget):
    _triggered = QtCore.pyqtSignal()

    def __init__(self, parent):
        super().__init__(parent)
        self.setLayout(QtWidgets.QVBoxLayout())
        
        # create lineedit
        self._lineedit = QtWidgets.QLineEdit(self)
        self._lineedit.setPlaceholderText("sum of digits")
        self._lineedit.setMinimumHeight(50)  
        self._label1 = QtWidgets.QLabel("write sum of digits:", self)

        # create radio buttons
        self.rb3 = QtWidgets.QRadioButton("3", self)
        self.rb4 = QtWidgets.QRadioButton("4", self)
        self.rb3.setMinimumHeight(50)
        self.rb4.setMinimumHeight(50)
        self.rb3.setChecked(True)
        button_layout = QtWidgets.QHBoxLayout()
        button_layout.addWidget(self.rb3)
        button_layout.addWidget(self.rb4)
        self._label2 = QtWidgets.QLabel("choose num of digits:", self)

        # create start button
        button = QtWidgets.QPushButton("RUN")
        button.setMinimumHeight(50)
        button.clicked.connect(lambda: self._triggered.emit())

        # set font
        font = QtGui.QFont()
        font.setPointSize(15)  
        self._lineedit.setFont(font)
        self.rb3.setFont(font)
        self.rb4.setFont(font)
        font2 = QtGui.QFont()
        font2.setPointSize(12)
        self._label1.setFont(font2)
        self._label2.setFont(font2)

        # add on layout
        self.layout().addWidget(self._label1) 
        self.layout().addWidget(self._lineedit)
        self.layout().addWidget(self._label2)
        self.layout().addLayout(button_layout)
        self.layout().addWidget(button)
        self.layout().addItem(QtWidgets.QSpacerItem(50, 50, QtWidgets.QSizePolicy.Policy.Minimum, QtWidgets.QSizePolicy.Policy.Expanding))
    
    def get_data(self):
        x = self._lineedit.text()
        try:
            x = int(x)
        except Exception as e:
            raise Exception("Enter numbers only")
        if x < 0:
            raise Exception("Sum of digits must be positive")
        y = 3 if self.rb3.isChecked() else 4
        return x, y

    def write_to_lineedit(self, x):
        self._lineedit.setText(str(x))