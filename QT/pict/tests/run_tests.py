import sys
import os
import argparse
from PyQt6 import QtWidgets
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import test1
from mainwin import MainWindow
from tester import tester

def run_tests(test_class=None, test_method=None):
    app = QtWidgets.QApplication(sys.argv)
    start_screen = MainWindow()
    start_screen.show()

    tester.init(0.1, 3, 1)
    if test_class and test_method:
        tester.test_names = [f"test1.{test_class}.{test_method}"]
    elif test_class:
        tester.test_case = getattr(test1, test_class)
    else:
        tester.test_module = test1

    tester.setapp(app)
    tester.start()

    app.exec()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run selective GUI tests')
    parser.add_argument('--class', '-c', dest='test_class',
                       help='Test class to run (e.g. "Test1")')
    parser.add_argument('--method', '-m', dest='test_method',
                       help='Specific test method to run (e.g. "test_window")')

    args = parser.parse_args()
    run_tests(test_class=args.test_class, test_method=args.test_method)