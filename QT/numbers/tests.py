import unittest
from mainwin import MainWindow
from tester import tester
from gui_communicator import guicom
from PyQt6 import QtWidgets

def n_lines(text):
    lines = text.toPlainText().split('\n')
    return len(lines)

def get_last_number(text):
    lines = text.toPlainText().split('\n')
    return lines[-1]

class Test(unittest.TestCase):
    def setUp(self):
        tester.start()
        self.mwin = tester.wait_window(MainWindow)

    def test_case3nubers(self):
        mwin = self.mwin
        self.assertIsInstance(mwin, MainWindow)

        self.assertEqual(mwin._right.rb3.isChecked(), True)
        self.assertEqual(mwin._right.rb4.isChecked(), False)

        win = tester.eval_and_wait_window(guicom.press_enter, (), QtWidgets.QMessageBox)
        self.assertEqual(win.text(), "Enter numbers only")
        tester.close_error_window()

        tester.eval_and_wait_true(mwin.write_to_lineedit, ("36"), "mwin._right._lineedit.text()=='36'", {'mwin': mwin})
        tester.eval_and_wait_true(guicom.press_enter, (), "mwin._left.toPlainText() == 'Total numbers: 0'", {'mwin': mwin})

        tester.eval_and_wait_true(mwin.write_to_lineedit, ("16"), "mwin._right._lineedit.text()=='16'", {'mwin': mwin})
        tester.eval_and_wait_true(guicom.press_enter, (), "get_last_number(mwin._left) == '970'", {'mwin': mwin, 'get_last_number': get_last_number})
        self.assertEqual(n_lines(mwin._left), 71)

        tester.eval_and_wait_true(mwin.write_to_lineedit, ("-27"), "mwin._right._lineedit.text()=='-27'", {'mwin': mwin})
        win = tester.eval_and_wait_window(guicom.press_enter, (), QtWidgets.QMessageBox)
        self.assertEqual(win.text(), "Sum of digits must be positive")
        tester.close_error_window()

    def test_case4numbers(self):
        mwin = self.mwin
        tester.eval_and_wait_true(guicom.set_checkbox, (mwin._right.rb4, True), 'mwin._right.rb4.isChecked()', {'mwin': mwin})
        tester.eval_and_wait_true(mwin.write_to_lineedit, ("one"), "mwin._right._lineedit.text()=='one'", {'mwin': mwin})
        win = tester.eval_and_wait_window(mwin._right._triggered.emit, (), QtWidgets.QMessageBox)
        self.assertEqual(win.text(), "Enter numbers only")
        tester.close_error_window()

        tester.eval_and_wait_true(mwin.write_to_lineedit, ("23"), "mwin._right._lineedit.text()=='23'", {'mwin': mwin})
        tester.eval_and_wait_true(mwin._right._triggered.emit, (), "get_last_number(mwin._left) == '9950'", {'mwin': mwin, 'get_last_number': get_last_number})
        
        tester.eval_and_wait_true(mwin.write_to_lineedit, ("36"), "mwin._right._lineedit.text()=='36'", {'mwin': mwin})
        tester.eval_and_wait_true(guicom.press_enter, (), "mwin._left.toPlainText() != 'Total numbers: 0'", {'mwin': mwin})
        self.assertEqual(n_lines(mwin._left), 3)
        tester.eval_and_wait_true(mwin.write_to_lineedit, ("37"), "mwin._right._lineedit.text()=='37'", {'mwin': mwin})
        tester.eval_and_wait_true(guicom.press_enter, (), "mwin._left.toPlainText() == 'Total numbers: 0'", {'mwin': mwin})

        tester.eval(quit, (mwin,))