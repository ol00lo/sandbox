from PyQt6 import QtWidgets, QtCore, QtGui

class RightControl(QtWidgets.QWidget):
    _triggered = QtCore.pyqtSignal()

    def __init__(self, parent):
        super().__init__(parent)
        self.setLayout(QtWidgets.QVBoxLayout())
        
        # create lineedit
        self._lineedit1 = QtWidgets.QLineEdit(self)
        self._lineedit1.setPlaceholderText("sum of digits")
        self._lineedit1.setMinimumHeight(50)  
        self._label1 = QtWidgets.QLabel("write sum of digits:", self)

        # create buttons 3 and 4
        self.button3 = QtWidgets.QPushButton("3", self)
        self.button4 = QtWidgets.QPushButton("4", self)
        self.button3.setMinimumHeight(50)
        self.button4.setMinimumHeight(50)

        # set options
        self.button3.setCheckable(True)
        self.button4.setCheckable(True)

        # button3 is  default
        self.button3.setChecked(True)  
        self.button3.setStyleSheet("background-color: lightblue;") 

        self.button3.clicked.connect(self.select_number)
        self.button4.clicked.connect(self.select_number)

        # create layout for 3 and 4
        button_layout = QtWidgets.QHBoxLayout()
        button_layout.addWidget(self.button3)
        button_layout.addWidget(self.button4)
        self._label2 = QtWidgets.QLabel("choose num of digits:", self)

        # create start button
        button = QtWidgets.QPushButton("START")
        button.setMinimumHeight(50)
        button.clicked.connect(lambda: self._triggered.emit())

        # set font
        font = QtGui.QFont()
        font.setPointSize(15)  
        self._lineedit1.setFont(font)
        font2 = QtGui.QFont()
        font2.setPointSize(12)
        self._label1.setFont(font2)
        self._label2.setFont(font2)

        # add on layout
        self.layout().addWidget(self._label1) 
        self.layout().addWidget(self._lineedit1)
        self.layout().addWidget(self._label2)
        self.layout().addLayout(button_layout)
        self.layout().addWidget(button)

    def select_number(self):
        if self.sender() == self.button3:
            self.button3.setChecked(True)
            self.button4.setChecked(False)
            self.button3.setStyleSheet("background-color: lightblue;")
            self.button4.setStyleSheet("background-color: none;")
        else:
            self.button4.setChecked(True)
            self.button3.setChecked(False)
            self.button4.setStyleSheet("background-color: lightblue;")
            self.button3.setStyleSheet("background-color: none;")