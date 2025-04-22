import time
import unittest
from PyQt6 import QtCore, QtWidgets
from .gui_communicator import guicom


class TestThread(QtCore.QThread):
    def __init__(self):
        super().__init__()

    # ========================= waiters
    def wait_window(self, win_t):
        """ wait till window of type win_t appear
            returns window object
        """
        return self.eval_and_wait_window(bool, True, win_t)

    def wait_signal(self, signal):
        return self.eval_and_wait_signal(bool, True, signal)

    def wait_focus(self, wdg_t):
        """ wait till widget of wdg_t gets focus
            returns this widget
        """
        return self.eval_and_wait_focus(bool, True, wdg_t)

    def wait_true(self, expr, kwargs={}):
        # I DONT NEED THIS
        """ waits expression is true
            uses eval(expr, kwargs)

            expr: str representation of expression
        """
        return self.eval_and_wait_true(bool, True, expr, kwargs)

    def wait_func_return(self, value, func, *args):
        """ waits until func(*args) == value """
        wrk = self._wait_func_return_wrk(value, func, *args)
        return self._eval_framework(bool, True, wrk)

    def wait_func_return_e(self, expr, func, *args):
        """ waits until expr(func(*args)) is True.
            Retruns func(*args) result or throws
        """
        wrk = self._wait_func_return_e_wrk(expr, func, *args)
        return self._eval_framework(bool, True, wrk)

    def close_error_window(self):
        """ waits for error window and closes it """
        self.wait_window(QtWidgets.QMessageBox)
        # guicom.click_button(w, "Ok")
        guicom.press_enter()

    # ========================= evaluation
    def eval(self, func, fargs):
        """ evaluates func(*fargs) in the testing thread
        """
        return self._eval_framework(func, fargs, lambda: True)

    def eval_and_wait_true(self, func, fargs, expr, kwargs):
        """ evaluate func(*fargs) and wait till expr(**kwargs) == True
        """
        return self._eval_framework(func, fargs,
                                    self._wait_true_wrk(expr, kwargs))

    def eval_and_wait_window(self, func, fargs, win_t):
        """ evaluate func(*fargs) and wait till window of type win_t appear
            returns window object
        """
        return self._eval_framework(func, fargs, self._wait_window_wrk(win_t))

    def eval_and_wait_signal(self, func, fargs, signal):
        """ evaluates func(*fargs) and waits for signal
            return signal object
        """
        ret = None

        def receiver(*args):
            nonlocal ret
            self.__wait_flag = True
            ret = args

        try:
            self.__wait_flag = False
            signal.connect(receiver)
            self._eval_framework(func, fargs, self._wait_flag_wrk())
            return ret
        except Exception:
            raise
        finally:
            self.__wait_flag = False
            signal.disconnect(receiver)

    def eval_and_confirm(self, func, fargs):
        """ evaluates func(*fargs) and hits confirm
        """
        self.__wait_flag = False

        def receiver():
            self.__wait_flag = True

        w = self.eval_and_wait_window(func, fargs, QtWidgets.QMessageBox)
        for but in w.buttons():
            if "Yes" in but.text() or "Ok" in but.text():
                break
        else:
            raise Exception("Confirm button was not found")
        but.clicked.connect(receiver)
        but.click()
        self._eval_framework(bool, True, self._wait_flag_wrk())

    def eval_and_wait_focus(self, func, fargs, wdg_t):
        return self._eval_framework(func, fargs, self._wait_focus_wrk(wdg_t))

    # ========================= emits
    def emit_and_wait_window(self, signal, sigargs, win_t):
        """ emits signal(*sigargs) and waits window of win_t type
            returns object of win_t
        """
        return self._emit_framework(signal, sigargs,
                                    self._wait_window_wrk(win_t))

    # ========================= call function
    def call(self, func, args=()):
        """ calls func(*args) in the gui thread
            returns the result of func
        """
        return guicom.efunc_return(func, *args)

    # == privates
    def _wait_true_wrk(self, expr, kwargs):
        def wrk():
            p = eval(expr, kwargs)
            if p is not True:
                print("p is not True, waiting")
                raise Exception("False expression {}".format(expr))
        return wrk

    def _wait_func_return_wrk(self, value, func, *args):
        def wrk():
            v = func(*args)
            if v == value:
                return True
            raise Exception(f"{v} != {value}")
        return wrk

    def _wait_func_return_e_wrk(self, expr, func, *args):
        def wrk():
            v = func(*args)
            if expr(v):
                return v
            raise Exception(f"{v} is not correct value")
        return wrk

    def _wait_focus_wrk(self, wdg_t):
        def wrk():
            w = self.qApp.focusWidget()
            if isinstance(w, wdg_t):
                return w
            raise Exception('focus widget was not set')
        return wrk

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

    def _eval_framework(self, func, funcargs, worker):
        if not isinstance(funcargs, tuple):
            funcargs = (funcargs,)
        return self._framework(guicom.efunc, ((func,) + funcargs), worker)

    def _emit_framework(self, sig, sigargs, worker):
        if not isinstance(sigargs, tuple):
            sigargs = (sigargs,)
        return self._framework(sig.emit, sigargs, worker)

    def _wait_window_wrk(self, tp):
        def wrk():
            for w in self.qApp.topLevelWidgets():
                if w.isVisible() and isinstance(w, tp):
                    self.eval_and_wait_true(self.qApp.setActiveWindow, (w,),
                                            'w.isActiveWindow()', {'w': w})
                    return w
            raise Exception("window {} failed to show up".format(tp))
        return wrk

    def _wait_flag_wrk(self):
        def wrk():
            if not self.__wait_flag:
                raise Exception("Flag was not set")
        return wrk

    def initialize_module(self, module, sleepsec=0.1, ntries=3, maxwait=1):
        self.__init(sleepsec, ntries, maxwait)
        self.test_module = module

    def initialize_testcase(self, testcase, sleepsec=0.1, ntries=3, maxwait=1):
        self.__init(sleepsec, ntries, maxwait)
        self.test_case = testcase

    def initialize_names(self, testnames, sleepsec=0.1, ntries=3, maxwait=1):
        self.__init(sleepsec, ntries, maxwait)
        self.test_names = testnames

    def run(self):
        loader = unittest.TestLoader()

        if self.test_module is not None:
            suite = loader.loadTestsFromModule(self.test_module)
        elif self.test_case is not None:
            suite = loader.loadTestsFromTestCase(self.test_case)
        elif self.test_names is not None:
            suite = loader.loadTestsFromNames(self.test_names)

        runner = unittest.TextTestRunner(verbosity=0)
        runner.run(suite)

    def __init(self, sleepsec, ntries, maxwait):
        self.test_module = None
        self.test_case = None

        self.__wait_flag = False
        self.sleepsec = sleepsec
        self.ntries = ntries
        self.maxwait = maxwait
    
    def setapp(self, app):
        self.qApp = app


tester = TestThread()