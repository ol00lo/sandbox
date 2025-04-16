import sys
from PyQt6 import QtWidgets
from mainwin import MainWindow
import tests
from tester import tester

app = QtWidgets.QApplication(sys.argv)
start_screen = MainWindow()
start_screen.show()

tester.initialize_module(tests)
tester.setapp(app)
tester.start()

app.exec()