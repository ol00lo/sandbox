from PyQt6 import QtWidgets, QtCore
import unittest
import time
from gui_communicator import guicom

class TestThread(QtCore.QThread):
    def __init__(self):
        super().__init__()
        self.test_module = None
        self.test_case = None
        self.test_names = None

    def initialize_module(self, module):
        self.init(0.1, 3, 1)
        self.test_module = module
    
    def run(self):
        loader = unittest.TestLoader()        
        if self.test_module is not None:
            suite = loader.loadTestsFromModule(self.test_module)
        elif self.test_case is not None:
            suite = loader.loadTestsFromTestCase(self.test_case)
        elif self.test_names is not None:
            suite = loader.loadTestsFromNames(self.test_names)
        else:
            raise Exception("nothing to load")
        runner = unittest.TextTestRunner(verbosity=0)
        runner.run(suite)
    
    def wait_window(self, window_cls):
        return self.eval_and_wait_window(bool, True, window_cls)
   
    def wait_signal(self, signal):
        return self.eval_and_wait_signal(bool, True, signal)

    def wait_focus(self, wdg_t):
        """ wait till widget of wdg_t gets focus
            returns this widget
        """
        return self.eval_and_wait_focus(bool, True, wdg_t)
    
    def close_error_window(self, w=None):
        """ waits for error window and closes it """
        self.wait_window(QtWidgets.QMessageBox)
        # guicom.click_button(w, "Ok")
        guicom.press_enter()

    def eval(self, func, fargs):
        """ evaluates func(*fargs) in the testing thread
        """
        return self._eval_framework(func, fargs, lambda: True)


    def eval_and_wait_window(self, func, fargs, win_t):
        """ evaluate func(*fargs) and wait till window of type win_t appear
            returns window object
        """
        return self._eval_framework(func, fargs, self._wait_window_wrk(win_t))

    def eval_and_wait_true(self, func, fargs, expr, kwargs):
        """ evaluate func(*fargs) and wait till expr(**kwargs) == True
        """
        return self._eval_framework(func, fargs,
                                    self._wait_true_wrk(expr, kwargs))
    
    def eval_and_wait_focus(self, func, fargs, wdg_or_wdg_t):
        return self._eval_framework(func, fargs, self._wait_focus_wrk(wdg_or_wdg_t))

    
    def _wait_window_wrk(self, tp):
        def wrk():
            for w in self.qApp.topLevelWidgets():
                if w.isVisible() and isinstance(w, tp):
                    # self.eval_and_wait_true(self.qApp.setActiveWindow, (w,),
                    #                        'w.isActiveWindow()', {'w': w})
                    return w
            raise Exception("window {} failed to show up".format(tp))
        return wrk

    def _wait_true_wrk(self, expr, kwargs):
        def wrk():
            p = eval(expr, kwargs)
            if p is not True:
                print("p is not True, waiting")
                raise Exception("False expression {}".format(expr))
        return wrk

    def _wait_focus_wrk(self, wdg_or_wdg_t):
        def wrk():
            w = self.qApp.focusWidget()
            if isinstance(wdg_or_wdg_t, type):
                wdg_t = wdg_or_wdg_t
                if isinstance(w, wdg_t):
                     return w
            else:
                if w is wdg_or_wdg_t:
                    return w
            raise Exception('focus widget was not set')
        return wrk

    def _eval_framework(self, func, funcargs, worker):
        if not isinstance(funcargs, tuple):
            funcargs = (funcargs,)
        return self._framework(guicom.efunc, ((func,) + funcargs), worker)

    def _framework(self, func, funcargs, worker):
        elast = None
        for i in range(self.ntries):
            try:
                func(*funcargs)
                s = 0
                while s < self.maxwait:
                    try:
                        time.sleep(self.sleepsec)
                        s += self.sleepsec
                        return worker()
                    except Exception as e:
                        elast = e
            except Exception as e:
                elast = e
        raise elast
    
    def init(self, sleepsec, ntries, maxwait):
        self.test_module = None
        self.test_case = None

        self.__wait_flag = False
        self.sleepsec = sleepsec
        self.ntries = ntries
        self.maxwait = maxwait
    
    def setapp(self, app):
        self.qApp = app

tester = TestThread()