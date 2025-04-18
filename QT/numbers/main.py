#!/usr/bin/env python3
import os
import sys
from PyQt6 import QtWidgets
import mainwin

def main(app):
    # create window
    mwin = mainwin.MainWindow()
    mwin.show()
    # start gui loop
    app.exec()


if __name__ == '__main__':
    # initialize qt application here to prevent
    # segmentation fault on exit
    qApp = QtWidgets.QApplication(sys.argv)
    main(qApp)
    sys.exit()