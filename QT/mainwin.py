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
        self._right.setMaximumWidth(400)

        self._left = QtWidgets.QTextEdit(self)
        self._left.setFontPointSize(18)
        self._left.setReadOnly(True)  
        self._left.setMinimumSize(150, 200)


        mainframe.layout().addWidget(self._left)
        mainframe.layout().addWidget(self._right)

    def _button_clicked(self):
        try:    
            digits_sum, num_digits = self._right.get_data()
            result = prog.generate_numbers(digits_sum, num_digits)  
            if result == []:
                self._left.setText("Total numbers: 0")
            else:
                self._left.setText(f"Automobile numbers:\n" + prog.to_string_numbers(result))

        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Error", str(e), QtWidgets.QMessageBox.StandardButton.Ok)

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key.Key_Return or event.key() == QtCore.Qt.Key.Key_Enter:
            self._right._triggered.emit()
            event.accept()    
        else:
            super().keyPressEvent(event)     