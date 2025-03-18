from PyQt6 import QtWidgets, QtGui, QtCore
import os
import cv2

class ImageInfo:
    def __init__(self, name, size, width, height):
        self.name = name
        self.width = width
        self.height = height
        self.size = size  
        self.area = (width * height) / 1_000_000  

class ImageModel(QtCore.QAbstractTableModel):
    columns = ["Name", "File Size (Bytes)", "Width", "Height", "Area (MP)"]

    def __init__(self, images, dir_path=None, parent=None):
        super().__init__(parent)
        self.images = images
        self.original_images = images.copy()  
        self.dir_path = dir_path

    def rowCount(self, parent):
        return len(self.images)

    def columnCount(self, parent):
        return len(self.columns)

    def data(self, index, role):
        if index.isValid() == False:
            return None
        if role == QtCore.Qt.ItemDataRole.DisplayRole or role == QtCore.Qt.ItemDataRole.EditRole:
            image_info = self.images[index.row()]
            if index.column() == 0:
                return image_info.name
            elif index.column() == 1:
                return image_info.size  
            elif index.column() == 2:
                return image_info.width
            elif index.column() == 3:
                return image_info.height
            elif index.column() == 4:
                return round(image_info.area, 2)  
        return None

    def headerData(self, section, orientation, role):
        if role == QtCore.Qt.ItemDataRole.DisplayRole:
            if orientation == QtCore.Qt.Orientation.Horizontal:
                return self.columns[section]
        return None

    def setData(self, index, value, role):
        if role == QtCore.Qt.ItemDataRole.EditRole and index.column() == 0:
            old_name = self.images[index.row()].name
            new_name = value.strip()
            if not new_name:
                return False
            if new_name != old_name:
                old_path = os.path.join(self.dir_path, old_name)
                new_path = os.path.join(self.dir_path, new_name)
                if os.path.exists(new_path):
                    QtWidgets.QMessageBox.warning(None, "Error", "File with this name already exists.")
                    return False

                try:
                    os.rename(old_path, new_path)
                    self.images[index.row()].name = new_name
                    self.dataChanged.emit(index, index, [QtCore.Qt.ItemDataRole.DisplayRole])
                    return True
                except Exception as e:
                    QtWidgets.QMessageBox.warning(None, "Error", f"Could not rename file: {e}")
                    return False
        return False

    def flags(self, index):
        if not index.isValid():
            return QtCore.Qt.ItemFlag.NoItemFlags
        return QtCore.Qt.ItemFlag.ItemIsEditable | QtCore.Qt.ItemFlag.ItemIsEnabled | QtCore.Qt.ItemFlag.ItemIsSelectable


    def filterImages(self, filter_text):
        self.beginResetModel()
        self.images = [img for img in self.original_images if filter_text.lower() in img.name.lower()]
        self.endResetModel()

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Image Table Viewer")
        self.setGeometry(100, 100, 800, 600)
        self.image_model = None

        self.initUI()

    def initUI(self):
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)

        self.layout = QtWidgets.QVBoxLayout(self.central_widget)

        self.filter_line_edit = QtWidgets.QLineEdit()
        self.filter_line_edit.setPlaceholderText("Filter by name...")
        self.filter_line_edit.textChanged.connect(self.on_filter_text_changed)
        self.layout.addWidget(self.filter_line_edit)

        self.table_view = QtWidgets.QTableView()
        self.table_view.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.DoubleClicked)
        self.layout.addWidget(self.table_view)

        self.load_images_button = QtWidgets.QPushButton("Load Images")
        self.load_images_button.clicked.connect(self.load_images)
        self.layout.addWidget(self.load_images_button)

    def load_images(self):
        folder = QtWidgets.QFileDialog.getExistingDirectory(self, "Select Folder")
        if folder:
            images = []
            for filename in os.listdir(folder):
                if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif')):
                    image_path = os.path.join(folder, filename)
                    image = cv2.imread(image_path)
                    if image is None:
                        continue  
                    size = os.path.getsize(image_path)  
                    height, width, _ = image.shape
                    images.append(ImageInfo(filename, size, width, height))

            if images:  
                self.image_model = ImageModel(images, dir_path=folder)
                self.table_view.setModel(self.image_model)

    def on_filter_text_changed(self, text):
        if self.image_model:
            self.image_model.filterImages(text)