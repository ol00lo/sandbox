import unittest
from PyQt6 import QtWidgets, QtCore
from mainwin import MainWindow
from tester import tester
from gui_communicator import guicom
from backend.state import State
from test_helper import create_test_folder


class Test1(unittest.TestCase):
    def test_case1(self):
        self.assertEqual(1, 1)
        mwin = tester.wait_window(MainWindow)
        self.assertIsInstance(mwin, MainWindow)
