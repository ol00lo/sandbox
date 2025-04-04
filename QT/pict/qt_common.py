from PyQt6 import QtGui, QtWidgets

SUPPORTED_IMAGE_EXTENSIONS = ('.png', '.jpg', '.jpeg', '.bmp')

def show_message(parent, title, message, is_error=False):
    dialog = QtWidgets.QMessageBox(parent)
    dialog.setWindowTitle(title)
    dialog.setText(message)

    if is_error:
        dialog.setWindowIcon(QtGui.QIcon(":/error"))
    else:
        dialog.setWindowIcon(QtGui.QIcon(":/right"))

    dialog.setStandardButtons(QtWidgets.QMessageBox.StandardButton.Ok)
    dialog.exec()