from PyQt6 import QtGui, QtCore, QtTest, QtWidgets
import time

def _call_in_gui(func):
    def ret(*args, **kwargs):
        guicom = args[0]
        caller = lambda: func(*args, **kwargs)
        guicom.emitter.emit(caller)
        time.sleep(guicom.sleepafter)

    return ret

class GuiCommunicator(QtCore.QObject):
    """ This class is used to simulate user activity on widgets.
        It has no wait functional.
        Use methods from test_thread.py for waiting.
    """
    emitter = QtCore.pyqtSignal(object)
    sleepafter = 0.0

    def __init__(self):
        super().__init__()
        self.emitter.connect(self._execute_fun)

    @_call_in_gui
    def press_key(self, key, wid=None, modifier=QtCore.Qt.KeyboardModifier.NoModifier):
        """ key: QtCore.Qt.Key_...
            wid: QtWidgets or None
        """
        QtTest.QTest.keyClick(wid, key, modifier)

    def press_enter(self):
        self.press_key(QtCore.Qt.Key.Key_Return)

    @_call_in_gui
    def type_text(self, text, wid=None, call_clear=False, focus=False):
        if focus and wid:
            self.focus_widget(wid)
        if wid is not None and call_clear:
            wid.clear()
        QtTest.QTest.keyClicks(wid, text)
        if focus and wid:
            self.no_focus_widget(wid)

    @_call_in_gui
    def set_combo_entry(self, cbwid, text=None, index=None):
        if text is not None:
            cbwid.setCurrentText(text)
        elif index is not None:
            cbwid.setCurrentIndex(index)

    @_call_in_gui
    def set_checkbox(self, chbox, flag):
        curval = chbox.checkStateSet() == QtCore.Qt.CheckState.Checked
        if curval != flag:
            chbox.toggle()

    @_call_in_gui
    def focus_widget(self, wid):
        wid.setFocus(QtCore.Qt.FocusReason.NoFocusReason)

    @_call_in_gui
    def no_focus_widget(self, wid):
        wid.clearFocus()

    @_call_in_gui
    def table_cell_dclick(self, row, col, tabwid):
        viewport = tabwid.viewport()
        w = tabwid.columnWidth(col)
        h = tabwid.rowHeight(row)
        pos = QtCore.QPoint(int(tabwid.columnViewportPosition(col) + w/2),
                            int(tabwid.rowViewportPosition(row) + h/2))
        QtTest.QTest.mouseClick(viewport, QtCore.Qt.LeftButton,
                                QtCore.Qt.NoModifier, pos)
        QtTest.QTest.mouseDClick(viewport, QtCore.Qt.LeftButton,
                                 QtCore.Qt.NoModifier, pos)

    @_call_in_gui
    def table_cell_set_value(self, row, col, tabwid, value):
        index = tabwid.model().index(row, col)
        tabwid.model().setData(index, value, QtCore.Qt.EditRole)

    @_call_in_gui
    def select_aview_item(self, aview, irow, icol):
        index = aview.model().index(irow, icol)
        com = QtCore.QItemSelectionModel.ClearAndSelect
        aview.selectionModel().select(index, com)

    def find_treemodel_index(self, model, row, column):
        data = []

        def get_index(root):
            nrows = model.rowCount(root)
            for i in range(nrows):
                index0 = model.index(i, 0, root)
                data.append(index0)
                get_index(index0)
        get_index(QtCore.QModelIndex())
        return data[row].siblingAtColumn(column)

    def find_treemodel_index_by_name(self, model, name, column):
        data = []

        def get_index(root):
            nrows = model.rowCount(root)
            for i in range(nrows):
                index0 = model.index(i, 0, root)
                data.append(index0)
                get_index(index0)
        get_index(QtCore.QModelIndex())

        fnd = None
        for d in data:
            v = d.data(QtCore.Qt.DisplayRole)
            if v[:v.rfind("(")-1].strip() == name:
                fnd = d
                break
        else:
            raise Exception(f'Failed to find tree index "{name} (...)"')
        return fnd.siblingAtColumn(column)

    @_call_in_gui
    def abstractview_item_click(self, index, view,
                                button=QtCore.Qt.MouseButton.LeftButton,
                                modifiers=QtCore.Qt.KeyboardModifier.NoModifier):
        viewport = view.viewport()
        pos = view.visualRect(index).center()
        QtTest.QTest.mouseClick(viewport, button, modifiers, pos)

    @_call_in_gui
    def click_button(self, wdg, button_cap):
        """ find button amoung child widgets
            with the given caption and clicks it
        """
        for w in wdg.children():
            if isinstance(w, QtWidgets.QPushButton):
                if button_cap.lower() == w.text().lower():
                    w.click()
                    return w
        for w in wdg.children():
            ret = self.click_button(w, button_cap)
            if ret is not None:
                return ret
        return None

    @_call_in_gui
    def click_mouse(self, wdg, pos,
                    button=QtCore.Qt.MouseButton.LeftButton,
                    modifiers=QtCore.Qt.KeyboardModifier.NoModifier):
        QtTest.QTest.mouseClick(wdg.viewport(), button, modifiers, pos)

    def drag_with_mouse(self, wdg, pos0, pos1,
                        button=QtCore.Qt.MouseButton.LeftButton,
                        modifiers=QtCore.Qt.KeyboardModifier.NoModifier,
                        n_steps=20):
        wdg = wdg.viewport()
        event = QtGui.QMouseEvent(QtCore.QEvent.MouseMove,
                                  pos0,
                                  wdg.mapToGlobal(pos0),
                                  button, button, modifiers)
        QtWidgets.QApplication.postEvent(wdg, event)
        QtTest.QTest.qWait(100)
        event = QtGui.QMouseEvent(QtCore.QEvent.MouseButtonPress,
                                  pos0,
                                  wdg.mapToGlobal(pos0),
                                  button, button, modifiers)
        QtWidgets.QApplication.postEvent(wdg, event)
        QtTest.QTest.qWait(100)
        for i in range(n_steps):
            t = float(i)/(n_steps-1)
            posi = (1 - t) * pos0 + t * pos1
            event = QtGui.QMouseEvent(QtCore.QEvent.MouseMove,
                                      posi,
                                      wdg.mapToGlobal(posi),
                                      button, button, modifiers)
            # should i use sendEvent here?
            QtWidgets.QApplication.postEvent(wdg, event)
            QtTest.QTest.qWait(5)
        event = QtGui.QMouseEvent(QtCore.QEvent.MouseButtonRelease,
                                  pos1,
                                  wdg.mapToGlobal(pos1),
                                  button, button, modifiers)
        QtWidgets.QApplication.postEvent(wdg, event)
        QtTest.QTest.qWait(100)

    def wheel_scroll(self, wdg, pos, num, modifiers=QtCore.Qt.KeyboardModifier.NoModifier):
        wdg = wdg.viewport()
        step = 1 if num > 0 else -1
        for i in range(abs(num)):
            event = QtGui.QWheelEvent(pos, wdg.mapToGlobal(pos),
                                      QtCore.QPoint(0, step), QtCore.QPoint(0, step),
                                      QtCore.Qt.NoButton, modifiers,
                                      QtCore.Qt.NoScrollPhase,
                                      False)
            # Do we have any problems with threads?
            # Seems like sendEvent uses the current thread, not a gui one.
            QtWidgets.QApplication.sendEvent(wdg, event)
            QtTest.QTest.qWait(5)

    def trigger_context_menu(self, wdg, pos, action_name):
        wdg = wdg.viewport()
        gpos = wdg.mapToGlobal(pos)
        event = QtGui.QContextMenuEvent(
                QtGui.QContextMenuEvent.Mouse,
                pos, gpos)

        # use postEvent due to threading
        QtWidgets.QApplication.postEvent(wdg, event)
        QtTest.QTest.qWait(10)

        action_name = action_name.split("/")
        for i in range(3):
            if i > 0:
                QtTest.QTest.qWait(50)
            for w in QtWidgets.QApplication.topLevelWidgets():
                if isinstance(w, QtWidgets.QMenu) and w.pos() == gpos:
                    break
            else:
                continue

            a = self._find_menu_action(w, action_name)
            if not a:
                continue

            self.efunc_return(a.trigger)
            QtTest.QTest.qWait(10)
            sig = spy.SignalSpy(w.aboutToHide)
            # sometimes menu is not closed for no reason.
            w.close()
            sig.wait_if_needed(500)
            return
        raise Exception(f"Failed to find actions: {action_name}")

    def _find_menu_action(self, menu, anames):
        for a in menu.actions():
            if a.text() == anames[0]:
                break
        else:
            return None
        if len(anames) == 1:
            return a
        else:
            return self._find_menu_action(a.menu(), anames[1:])

    @_call_in_gui
    def efunc(self, method, *args):
        method(*args)

    def efunc_return(self, method, *args):
        """ Waits until method returns something """
        result_init = object()
        result = result_init

        def func():
            nonlocal result
            result = method(*args)

        self.emitter.emit(func)

        time.sleep(self.sleepafter)

        for _ in range(10):
            if result is not result_init:
                return result
            QtTest.QTest.qWait(100)
        raise Exception("Failed to wait for return")

    def _execute_fun(self, fun):
        fun()


guicom = GuiCommunicator()
