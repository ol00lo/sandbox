import unittest
from PyQt6 import QtWidgets, QtCore, QtGui
from mainwin import MainWindow
from tester import tester
from gui_communicator import guicom
from backend.state import State
from backend.box import Box
from test_helper import create_test_folder
import os

class Test1(unittest.TestCase):
    def __init__(self, methodName = "runTest"):
        super().__init__(methodName)
        self.mwin = tester.wait_window(MainWindow)
        self.csv_name = "bbox_output.csv"
        self.path = os.path.dirname(os.path.abspath(__file__)) + "/test_images/"

    def setUp(self):
        self.widget = None
        for root, dirs, files in os.walk(self.path, topdown=False):
            for file in files:
                os.remove(os.path.join(root, file))
        create_test_folder()
        self.mwin.open_folder(self.path)

    def test_gui(self):
        self.assertEqual(1, 1)
        mwin = self.mwin
        self.assertIsInstance(mwin, MainWindow)

        #open
        self.path = "test_images/bbox_output.csv"
        self.assertEqual(State().model.get_len(), 10)

        # draw
        tester.eval(mwin.table_viewer.update_row_focus, (None, 0, 0) )
        self.widget = mwin.image_model_viewer

        viewer_size = mwin.image_model_viewer.size()
        start_point1 = QtCore.QPoint(viewer_size.width() // 5, viewer_size.height() // 5)
        start_point2 = QtCore.QPoint(viewer_size.width() // 4, viewer_size.height() // 4)
        end_point1 = QtCore.QPoint(viewer_size.width() // 2, viewer_size.height() // 2)
        end_point2 = QtCore.QPoint(viewer_size.width() // 3, viewer_size.height() // 3)
        new_end_point1 = QtCore.QPoint(int(viewer_size.width() // 1.5), int(viewer_size.height() // 1.5))

        self.draw_box(start_point1, end_point1)
        self.draw_box(start_point2, end_point2)

        #resize
        old_string = self.get_string_from_file(self.path, 1)
        tester.eval_and_wait_true(guicom.drag_with_mouse, (self.widget, end_point1, new_end_point1, QtCore.Qt.MouseButton.LeftButton),
                                  's() == True', {'s': lambda: self.get_string_from_file(self.path, 2) != old_string})

        # delete
        tester.eval_and_wait_window(guicom.click_mouse, 
                                     (self.widget, new_end_point1, QtCore.Qt.MouseButton.RightButton), QtWidgets.QMessageBox)
        guicom.press_key(QtCore.Qt.Key.Key_Left)
        guicom.press_enter()

        # delete image
        tester.eval(mwin.table_viewer.update_row_focus, (None, 0, 0) )
        tester.eval_and_wait_true(
            guicom.press_key, (QtCore.Qt.Key.Key_Delete), 'a() == 0', {'a': State().box_saver.all_boxes_count}
        )

        # rescale window
        before = mwin.image_model_viewer.transform().m11()
        tester.eval_and_wait_true(mwin.resize, (1100, 800), 'a() == True', 
                                  {'a': lambda: mwin.image_model_viewer.transform().m11()!= before})

    def test_delete_img(self):
        box = Box("AAA", QtCore.QRectF(10, 10, 20, 20))
        State().actions["CreateBox"].do(box, "test9.jpg")

        index = State().model.index_by_imagename("test9.jpg")
        State().actions["DeleteImage"].delete(index)

        # check that image was deleted
        self.assertEqual(State().model.rowCount(), 9)
        self.assertIsNone(State().model.index_by_imagename("test9.jpg"))

        # check no entry for boxes in csv
        with open(os.path.join(self.path, self.csv_name), "r") as f:
            lines = f.readlines()
            self.assertEqual(len(lines), 1)

    def test_change_box(self):
        # abs path
        folder = os.path.abspath('test_images')
        csv_path = os.path.join(folder, self.csv_name)
        img = "test9.jpg"
        State().actions["LoadImages"].openfolder(folder)

        box = Box("AAA", QtCore.QRectF(10, 10, 20, 20))
        State().actions["CreateBox"].do(box, img)
        before = self.get_string_from_file(csv_path,  1)
        new_box = Box("AAA", QtCore.QRectF(10, 10, 30, 40))
        State().actions["ResizeBox"].do(box, new_box, os.path.join(folder, img))
        after = self.get_string_from_file(csv_path,1)
        self.assertNotEqual(before, after)

        State().actions["DeleteBox"].do(box, os.path.join(folder, img))
        self.assertEqual(State().box_saver.box_count(img), 1)

        State().actions["DeleteBox"].do(new_box, os.path.join(folder, img))
        self.assertEqual(State().box_saver.box_count(img), 0)

    def test_window(self):
        before = self.mwin.image_model_viewer.transform().m11()
        tester.eval_and_wait_true(self.mwin.resize, (900, 700), 'a() == True',
                                  {'a': lambda: self.mwin.image_model_viewer.transform().m11()!= before})



    def draw_box(self, start_pos, end_pos):
        tester.eval_and_wait_window(guicom.drag_with_mouse, (self.widget, start_pos, end_pos, QtCore.Qt.MouseButton.LeftButton),  QtWidgets.QInputDialog)
        guicom.type_text("a")
        guicom.press_enter()

    def get_string_from_file(self, path:str, N:int):
        with open(path, "r") as f:
           return f.readlines()[N]
