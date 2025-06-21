#!/usr/bin/env python3
import os
import sys
from PyQt6 import QtWidgets
import mainwin
import resources

def main(app, initial_dir=None):
    # create window

    resources.qInitResources()
    mwin = mainwin.MainWindow()

    if initial_dir:
        mwin.open_folder(initial_dir)

    mwin.show()

    # start gui loop
    app.exec()


if __name__ == '__main__':
    # initialize qt application here to prevent
    # segmentation fault on exit
    qApp = QtWidgets.QApplication(sys.argv)

    initial_dir = None
    if len(sys.argv) > 1:
        initial_dir = sys.argv[1]
        if not os.path.isdir(initial_dir):
            print(f"Error: '{initial_dir}' is not a valid directory", file=sys.stderr)
            initial_dir = None

    main(qApp, initial_dir)
    sys.exit()