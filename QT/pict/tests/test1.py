import unittest
from PyQt6 import QtWidgets, QtCore
from mainwin import MainWindow
from tester import tester
from gui_communicator import guicom
from backend.state import State
from backend.box import Box
from test_helper import create_test_folder
import os

class Test1(unittest.TestCase):
    def test_gui(self):
        self.assertEqual(1, 1)
        mwin = tester.wait_window(MainWindow)
        self.assertIsInstance(mwin, MainWindow)

        #open
        tester.eval(create_test_folder, ())
        self.path = "tests/test_images/bbox_output.csv"
        if os.path.exists(self.path):
            os.remove(self.path)
        tester.eval_and_wait_true(mwin.open_folder, (), "len() == 10", {'len': State().model.get_len}) 

        # draw
        tester.eval(mwin.table_viewer.update_row_focus, (None, 0, 0) )
        self.widget = mwin.image_model_viewer
        self.draw_box(QtCore.QPoint(100, 100), QtCore.QPoint(300, 300), 1)
        self.draw_box(QtCore.QPoint(200, 200), QtCore.QPoint(250, 250), 2)

        #resize
        drag_start = QtCore.QPoint(301, 301)
        drag_end = QtCore.QPoint(400, 400)
        old_string = self.get_string_from_file(1)
        tester.eval_and_wait_true(guicom.drag_with_mouse, (self.widget, drag_start, drag_end, QtCore.Qt.MouseButton.LeftButton),
                                  's() == True', {'s': lambda: self.get_string_from_file(2) != old_string})

        # delete
        tester.eval_and_wait_window(guicom.click_mouse, 
                                     (self.widget, QtCore.QPoint(400, 301), QtCore.Qt.MouseButton.RightButton), QtWidgets.QMessageBox)
        tester.eval_and_wait_true(self.action, (), 'a() == 1', {'a': State().box_saver.bbox_data.__len__})

        # delete image
        tester.eval(mwin.table_viewer.update_row_focus, (None, 0, 0) )
        tester.eval_and_wait_true(
            guicom.press_key, (QtCore.Qt.Key.Key_Delete), 'a() == 0', {'a': State().box_saver.bbox_data.__len__}
        )

        # rescale window
        before = mwin.image_model_viewer.transform().m11()
        tester.eval_and_wait_true(mwin.resize, (1100, 800), 'a() == True', 
                                  {'a': lambda: mwin.image_model_viewer.transform().m11()!= before})

    def test_delete_img(self):
        create_test_folder()
        State().actions["LoadImages"].openfolder("tests/test_images")
        box = Box("AAA", QtCore.QRectF(100, 100, 200, 200))
        State().actions["CreateBox"].do(box, "test0.jpg")

        index = State().model.index_by_imagename("test0.jpg")
        State().actions["DeleteImage"].delete(index)

        # check that image was deleted
        self.assertEqual(State().model.rowCount(), 9)
        self.assertIsNone(State().model.index_by_imagename("test0.jpg"))

        # check no entry for boxes in csv
        with open("tests/test_images/bbox_output.csv", "r") as f:
            lines = f.readlines()
            self.assertEqual(len(lines), 1)

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
