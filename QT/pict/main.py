#!/usr/bin/env python3
import os
import sys
from PyQt6 import QtWidgets
import mainwin
import resources

def main():
    app = QtWidgets.QApplication(sys.argv)

    resources.qInitResources()
    mwin = mainwin.MainWindow()
    mwin.show()

    sys.exit(app.exec())

if __name__ == '__main__':
    main()