import unittest
from mainwin import MainWindow
from tester import tester
from gui_communicator import guicom
from PyQt6 import QtWidgets, QtCore

class Test(unittest.TestCase):
    def setUp(self):
        self.mwin = tester.wait_window(MainWindow)

    def test_case3numbers(self):
        mwin = self.mwin
        self.assertIsInstance(mwin, MainWindow)

        tester.eval_and_wait_true(guicom.set_checkbox, (mwin._right.rb3, True), 'isChecked3()', {'isChecked3': mwin._right.rb3.isChecked})
        self.assertEqual(mwin._right.rb4.isChecked(), False)

        # empty input
        win = tester.eval_and_wait_window(self.press_run, (), QtWidgets.QMessageBox)
        self.assertEqual(win.text(), "Enter integer numbers only")
        tester.close_error_window()

        tester.eval_and_wait_true(self.write_text, ("36",), "get_text()=='36'", {'get_text': mwin._right._lineedit.text})
        tester.eval_and_wait_true(self.press_run, (), "res() == 'Total numbers: 0'", {'res': mwin._left.toPlainText})

        tester.eval_and_wait_true(self.write_text, ("16",), "get_text()=='16'", {'get_text': mwin._right._lineedit.text})
        tester.eval_and_wait_true(self.press_run, (), "last_number() == '970'", {'mwin': mwin, 'last_number': self.get_last_number})
        self.assertEqual(self.n_lines(), 71)

        # negative input
        tester.eval_and_wait_true(self.write_text, ("-27",), "get_text()=='-27'", {'get_text': mwin._right._lineedit.text})
        win = tester.eval_and_wait_window(self.press_run, (), QtWidgets.QMessageBox)
        self.assertEqual(win.text(), "Sum of digits must be positive")
        tester.close_error_window()

        # zero input
        tester.eval_and_wait_true(self.write_text, ("0",), "get_text()=='0'", {'get_text': mwin._right._lineedit.text})
        tester.eval_and_wait_true(self.press_run, (), "last_number() == '000'", {'last_number': self.get_last_number})

        # with space inside
        tester.eval_and_wait_true(self.write_text, ("1 1",), "get_text()=='1 1'", {'get_text': mwin._right._lineedit.text})
        win = tester.eval_and_wait_window(self.press_run, (), QtWidgets.QMessageBox)
        self.assertEqual(win.text(), "Enter integer numbers only")
        tester.close_error_window()

        # with space at the end
        tester.eval_and_wait_true(self.write_text, ("11  ",), "get_text()=='11  '", {'get_text': mwin._right._lineedit.text})
        tester.eval_and_wait_true(self.press_run, (), "res() != 'Total numbers: 0'", {'res': mwin._left.toPlainText})

        # float number
        tester.eval_and_wait_true(self.write_text, ("3.6",), "get_text()=='3.6'", {'get_text': mwin._right._lineedit.text})
        win = tester.eval_and_wait_window(self.press_run, (), QtWidgets.QMessageBox)
        self.assertEqual(win.text(), "Enter integer numbers only")
        tester.close_error_window()

        # <-/->
        tester.eval_and_wait_focus(self.write_text, ("\t",), QtWidgets.QRadioButton)
        tester.eval_and_wait_true(guicom.press_key, (QtCore.Qt.Key.Key_Right),  'isChecked4()', {'isChecked4': mwin._right.rb4.isChecked})  

    def test_case4numbers(self):
        mwin = self.mwin

        # not a number
        tester.eval_and_wait_true(guicom.set_checkbox, (mwin._right.rb4, True), 'isChecked4()', {'isChecked4': mwin._right.rb4.isChecked})
        tester.eval_and_wait_true(mwin.write_to_lineedit, ("one",), "get_text() =='one'", {'get_text': mwin.get_from_lineedit})
        win = tester.eval_and_wait_window(mwin._right._triggered.emit, (), QtWidgets.QMessageBox)
        self.assertEqual(win.text(), "Enter integer numbers only")
        tester.close_error_window()

        tester.eval_and_wait_true(mwin.write_to_lineedit, ("23",), "get_text() =='23'", {'get_text': mwin.get_from_lineedit})
        tester.eval_and_wait_true(mwin._right._triggered.emit, (), "last_number() == '9950'", {'mwin': mwin, 'last_number': self.get_last_number})
        
        self.mwin._right.button.setFocus()
        tester.eval_and_wait_true(mwin.write_to_lineedit, ("36",), "get_text() =='36'", {'get_text': mwin.get_from_lineedit})
        tester.eval_and_wait_true(guicom.press_enter, (), "res() != 'Total numbers: 0'", {'res': mwin._left.toPlainText})
        self.assertEqual(self.n_lines(), 3)
        tester.eval_and_wait_true(mwin.write_to_lineedit, ("37",), "get_text() =='37'", {'get_text': mwin.get_from_lineedit})
        tester.eval_and_wait_true(guicom.press_enter, (), "res() == 'Total numbers: 0'", {'res': mwin._left.toPlainText})
        
        # with tab
        tester.eval_and_wait_focus(self.write_text, ("0\t",), QtWidgets.QRadioButton)
        tester.eval_and_wait_true(self.press_run, (), "last_number() == '0000'", {'last_number': self.get_last_number})
    
    def n_lines(self):
        lines = self.mwin._left.toPlainText().split('\n')
        return len(lines)

    def get_last_number(self):
        lines = self.mwin._left.toPlainText().split('\n')
        return lines[-1]
    
    def press_run(self):
        self.mwin._right.button.setFocus()
        guicom.press_enter()

    def write_text(self, text):
        self.mwin._right._lineedit.clear()
        self.mwin._right._lineedit.setFocus()
        guicom.type_text(text)