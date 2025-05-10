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
        tester.eval(create_test_folder, ())
        tester.eval_and_wait_true(mwin.open_folder, (), "len() == 10", {'len': State().model.get_len}) 

        tester.eval(mwin.table_viewer.update_row_focus, (None, 0, 0) )
        start_pos = QtCore.QPoint(100, 100)
        end_pos = QtCore.QPoint(300, 300)
        widget = mwin.image_model_viewer
        tester.eval_and_wait_window(guicom.drag_with_mouse, (widget, start_pos, end_pos, QtCore.Qt.MouseButton.LeftButton),  QtWidgets.QInputDialog)
        guicom.type_text("a")
        tester.eval_and_wait_true(guicom.press_enter,(), 'text()!=""', {'text': self.get_string_from_file})

    def get_string_from_file(self):
        with open("bbox_output.csv", "r") as f:
           return f.read()
