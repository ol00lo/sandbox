import sys
import os
from PyQt6 import QtWidgets
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import test1
from mainwin import MainWindow
from tester import tester

def run_tests():
    app = QtWidgets.QApplication(sys.argv)
    start_screen = MainWindow()
    start_screen.show()

    tester.initialize_module(test1)
    tester.setapp(app)
    tester.start()

    app.exec()

if __name__ == '__main__':
    run_tests()