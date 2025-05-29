import unittest
from PyQt6 import QtWidgets, QtCore
from mainwin import MainWindow
from tester import tester
from gui_communicator import guicom
from backend.state import State
from test_helper import create_test_folder
import os

class Test1(unittest.TestCase):
    def test_case1(self):
        self.assertEqual(1, 1)
        mwin = tester.wait_window(MainWindow)
        self.assertIsInstance(mwin, MainWindow)
        tester.eval(create_test_folder, ())
        self.path = "tests/test_images/bbox_output.csv"
        if os.path.exists(self.path):
            os.remove(self.path)
        tester.eval_and_wait_true(mwin.open_folder, (), "len() == 10", {'len': State().model.get_len}) 

        tester.eval(mwin.table_viewer.update_row_focus, (None, 0, 0) )
        self.widget = mwin.image_model_viewer
        self.draw_box(QtCore.QPoint(100, 100), QtCore.QPoint(300, 300), 1)
        self.draw_box(QtCore.QPoint(200, 200), QtCore.QPoint(250, 250), 2)

        drag_start = QtCore.QPoint(302, 302)
        drag_end = QtCore.QPoint(400, 400)
        old_string = self.get_string_from_file(1)
        tester.eval_and_wait_true(guicom.drag_with_mouse, (self.widget, drag_start, drag_end, QtCore.Qt.MouseButton.LeftButton),
                                  's() == True', {'s': lambda: self.get_string_from_file(2) != old_string})

        tester.eval_and_wait_window(guicom.click_mouse, (self.widget, QtCore.QPoint(400, 300), QtCore.Qt.MouseButton.RightButton), QtWidgets.QMessageBox)
        tester.eval_and_wait_true(self.action, (), 'a() == 1', {'a': State().box_saver.bbox_data.__len__})


    def action(self):
        guicom.press_key(QtCore.Qt.Key.Key_Left)
        guicom.press_enter()

    def draw_box(self, start_pos, end_pos, N):
        tester.eval_and_wait_window(guicom.drag_with_mouse, (self.widget, start_pos, end_pos, QtCore.Qt.MouseButton.LeftButton),  QtWidgets.QInputDialog)
        guicom.type_text("a")
        tester.eval_and_wait_true(guicom.press_enter,(), f'text({N})!=""', {'text': self.get_string_from_file})

    def get_string_from_file(self, N:int):
        with open(self.path, "r") as f:
           return f.readlines()[N]
