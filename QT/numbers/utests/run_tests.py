import sys
from PyQt6 import QtWidgets
from mainwin import MainWindow
import utests.tests
from .tester import tester

# run
# python -m unittest utests.run_tests

app = QtWidgets.QApplication(sys.argv)
start_screen = MainWindow()
start_screen.show()

params = sys.argv[2:]
if not params:
    tester.initialize_module(utests.tests)
else:
    names = ["utests.tests." + x for x in params]
    tester.initialize_names(names)

tester.setapp(app)
tester.start()

app.exec()