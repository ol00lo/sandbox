from PyQt6 import QtWidgets, QtGui
import os
import cv2
from model import ImageModel, ImageInfo, ImageProxyModel

class ImageScene(QtWidgets.QGraphicsScene):
    def __init__(self, parent=None):
        super().__init__(parent)

    def display_image(self, image_path):
        self.clear()
        pixmap = QtGui.QPixmap(image_path)
        pixmap_item = self.addPixmap(pixmap)
        self.setSceneRect(pixmap_item.boundingRect())

class ImageViewer(QtWidgets.QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.image_scene = ImageScene(self)
        self.setScene(self.image_scene)

    def display_image(self, image_path):
        self.image_scene.display_image(image_path)


class ImageModelViewer:
    def __init__(self, parent):
        self.parent = parent
        self.setup_ui()

    def setup_ui(self):
        self.filter_line_edit = QtWidgets.QLineEdit()
        self.filter_line_edit.setPlaceholderText("Filter by name...")
        self.filter_line_edit.textChanged.connect(self.on_filter_text_changed)

        self.table_view = QtWidgets.QTableView()
        self.table_view.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.DoubleClicked)

        self.load_images_button = QtWidgets.QPushButton("Load Images")
        self.load_images_button.clicked.connect(self.load_images)

        self.image_viewer = ImageViewer(self.parent)

        layout = QtWidgets.QHBoxLayout(self.parent.central_widget)

        left_layout = QtWidgets.QVBoxLayout()
        left_layout.addWidget(self.filter_line_edit)
        left_layout.addWidget(self.table_view)
        left_layout.addWidget(self.load_images_button)

        self.table_view.setMaximumWidth(400)

        layout.addLayout(left_layout)
        layout.addWidget(self.image_viewer)

        layout.setStretch(0, 0)  
        layout.setStretch(1, 1) 


    def load_images(self):
        folder = QtWidgets.QFileDialog.getExistingDirectory(self.parent, "Select Folder")
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
                self.image_proxy_model = ImageProxyModel(self.parent)
                self.image_proxy_model.setSourceModel(self.image_model)

                self.table_view.setModel(self.image_proxy_model)
                self.table_view.selectionModel().selectionChanged.connect(self.on_selection_changed)

                for column_index in range(len(self.image_model.columns)):
                    self.table_view.horizontalHeader().setSectionResizeMode(column_index, QtWidgets.QHeaderView.ResizeMode.Stretch)

                self.table_view.horizontalHeader().sectionDoubleClicked.connect(self.sort_table)

    def sort_table(self, index):
        self.image_proxy_model.sort(index)

    def on_filter_text_changed(self, text):
        if self.image_model:
            self.image_model.filterImages(text)

    def on_selection_changed(self, selected, deselected):
        index = self.table_view.selectionModel().currentIndex()
        if index.isValid() and index.column() == 0:
            image_info = self.image_model.images[index.row()]
            self.display_image(image_info.name)

    def display_image(self, image_name):
        image_path = os.path.join(self.image_model.dir_path, image_name)
        self.image_viewer.display_image(image_path)
