from PyQt6 import QtCore
import os
import shutil
import imagesize

class ImageInfo:
    def __init__(self, path):
        self.set_properties(path)

    def set_properties(self, path):
        if not os.path.exists(path):
            raise Exception("File does not exist.")
        self.name = os.path.basename(path)
        self.width, self.height = imagesize.get(path)
        self.size = os.path.getsize(path)
        self.area = (self.width * self.height) / 1_000_000  

class TableModel(QtCore.QAbstractTableModel):
    columns = ["Name", "File Size", "Width", "Height", "Area"]

    def __init__(self, images=[], dir_path=None, parent=None):
        super().__init__(parent)
        self.set_data(images, dir_path)

    def set_data(self, images, dir_path=None):
        self.layoutAboutToBeChanged.emit()
        self.images = images
        self.dir_path = dir_path
        self.layoutChanged.emit()

    def add_image(self, image_info, source_path):
        if not self.dir_path:
            raise Exception("Select folder.")

        destination_path = os.path.join(self.dir_path, image_info.name)
        if os.path.exists(destination_path):
            raise Exception ("File with this name already exists.")

        shutil.copy2(source_path, destination_path)

        self.images.append(image_info)

        self.beginInsertRows(QtCore.QModelIndex(), len(self.images) - 1, len(self.images) - 1)
        self.endInsertRows()

        self.dataChanged.emit(self.index(len(self.images) - 1, 0), self.index(len(self.images) - 1, 0))

    def rowCount(self, parent):
        return len(self.images)

    def columnCount(self, parent):
        return len(self.columns)

    def data(self, index, role):
        if role in [QtCore.Qt.ItemDataRole.DisplayRole, QtCore.Qt.ItemDataRole.EditRole, QtCore.Qt.ItemDataRole.ToolTipRole]:
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
        if role == QtCore.Qt.ItemDataRole.DisplayRole and orientation == QtCore.Qt.Orientation.Horizontal:
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
                    return False

                os.rename(old_path, new_path)
                self.images[index.row()].name = new_name
                self.dataChanged.emit(index, index, [QtCore.Qt.ItemDataRole.DisplayRole])
                return True
        return False

    def clear_data(self):
        self.beginResetModel()
        self.images.clear()
        self.dir_path = ""
        self.endResetModel()

    def flags(self, index):
        ret = super().flags(index)

        if index.column() == 0: 
            ret |= QtCore.Qt.ItemFlag.ItemIsEditable
        ret |= QtCore.Qt.ItemFlag.ItemIsEnabled
        ret |= QtCore.Qt.ItemFlag.ItemIsSelectable
        return ret
    def get_len(self):
        return len(self.images)
