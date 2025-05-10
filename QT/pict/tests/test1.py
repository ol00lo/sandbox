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

        tester.eval_and_wait_true(self.type_text_and_press_enter,("a"), 'len() == 1', {'len': mwin.image_model_viewer.image_model.n_boxes})

        tester.eval_and_wait_window(guicom.click_mouse, (widget, QtCore.QPoint(200, 200), QtCore.Qt.MouseButton.RightButton), QtWidgets.QMessageBox)
        tester.eval_and_wait_true(guicom.press_enter,(), 'len() == 0', {'len': mwin.image_model_viewer.image_model.n_boxes})

    def type_text_and_press_enter(self, text: str):
        guicom.type_text(text)
        guicom.press_enter()
