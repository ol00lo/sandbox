import sys
import os
from PyQt6 import QtWidgets
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import test1
from mainwin import MainWindow
from tester import tester

# python tests/run_tests.py

app = QtWidgets.QApplication(sys.argv)
start_screen = MainWindow()
start_screen.show()

tester.initialize_module(test1)
tester.setapp(app)
tester.start()

app.exec()