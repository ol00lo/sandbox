from PyQt6 import QtGui, QtWidgets

SUPPORTED_IMAGE_EXTENSIONS = ('.png', '.jpg', '.jpeg', '.bmp')
SUPPORTED_IMAGE_FILTER = 'Images (' + ' '.join([f'*{x}' for x in SUPPORTED_IMAGE_EXTENSIONS]) + ')'

def show_message(title = "ERROR", message = "", is_error=False):
    dialog = QtWidgets.QMessageBox()
    dialog.setWindowTitle(title)
    dialog.setText(message)

    if is_error:
        dialog.setIcon(QtWidgets.QMessageBox.Icon.Critical)
        dialog.setWindowIcon(QtGui.QIcon(":/error"))
    else:
        dialog.setIcon(QtWidgets.QMessageBox.Icon.Information)
        dialog.setWindowIcon(QtGui.QIcon(":/right"))

    dialog.setStandardButtons(QtWidgets.QMessageBox.StandardButton.Ok)
    dialog.exec()