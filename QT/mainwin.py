from PyQt6 import QtWidgets, QtCore
import prog
from rightcontrol import RightControl 

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.resize(500, 300)
        self.setWindowTitle("Automobile numbers")

        mainframe = QtWidgets.QFrame(self)
        mainframe.setLayout(QtWidgets.QHBoxLayout())
        self.setCentralWidget(mainframe)

        self._right = RightControl(self)
        self._right._triggered.connect(self._button_clicked)

        self._left = QtWidgets.QTextEdit(self)
        self._left.setFontPointSize(18)
        self._left.setReadOnly(True)  

        mainframe.layout().addWidget(self._left)
        mainframe.layout().addWidget(self._right)

    def _button_clicked(self):
        try:    
            x = self._right._lineedit1.text()
            y = 3 if self._right.button3.isChecked() else 4
            if not x.isdigit():  
                raise Exception("Enter numbers only")
            x = int(x)

            result = prog.generate_numbers(x, y)  
            if result == []:
                self._left.setText("No automobile numbers")
            else:
                self._left.setText(f"Automobile numbers:" + prog.to_string_numbers(result))

        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", str(e), QtWidgets.QMessageBox.StandardButton.Ok)

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key.Key_Return or event.key() == QtCore.Qt.Key.Key_Enter:
            self._right._triggered.emit()    