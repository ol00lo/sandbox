import os
import shutil
import csv
from .box import Box

class BackUp:
    BACKUP_PREFIX = "~backup_"
    def __init__(self):
        self.backup_dir = None

    def set_backup_dir(self, directory):
        import time
        self.cleanup_old_backups(directory)
        dir_name = f"{self.BACKUP_PREFIX}{int(time.time())}"
        self.backup_dir = directory + dir_name
        os.makedirs(self.backup_dir, exist_ok=True)

        return self.backup_dir

    def save_file(self, file_path, boxes, folder = None):
        if folder:
            curr_dir = os.path.join(self.backup_dir, folder)
            os.makedirs(curr_dir, exist_ok=True)
        else:
            curr_dir = self.backup_dir
        backup_list = []
        file_name = os.path.basename(file_path)
        dest_path = os.path.join(curr_dir, file_name)
        csv_path = os.path.join(curr_dir, f"{file_name.split('.')[0]}.csv")
        shutil.move(file_path, dest_path)
        backup_list.append(dest_path)
        with open(csv_path, 'a', newline='') as csv_file:
            writer = csv.writer(csv_file)
            for box in boxes:
                box.write(writer, file_name)
        backup_list.append(csv_path)
        return backup_list

    def return_file(self, file_name, new_path):
        file_path = os.path.join(self.backup_dir, file_name)
        shutil.move(file_path, new_path)

    def get_boxes_on_image(self, file_name):
        csv_path = os.path.join(self.backup_dir, f"{file_name.split('.')[0]}.csv")
        boxes = []
        if os.path.exists(csv_path):
            with open(csv_path, 'r', newline='') as csv_file:
                reader = csv.reader(csv_file)
                for row in reader:
                    box = Box()
                    box.read_and_ret_name(row)
                    boxes.append(box)
        return boxes

    def delete_file(self, file_name):
        file_path = os.path.join(self.backup_dir, file_name)
        if os.path.exists(file_path):
            os.remove(file_path)

    def delete_files(self, file_names):
        for file_name in file_names:
            self.delete_file(file_name)

    def clear_all(self):
        if not self.backup_dir:
            return
        try:
            shutil.rmtree(self.backup_dir)
        except Exception as e:
            print(f"Failed to delete current backup dir {self.backup_dir}: {e}")

    def cleanup_old_backups(self, target_dir):
        for item_name in os.listdir(target_dir):
            if item_name.startswith(self.BACKUP_PREFIX):
                try:
                    item_path = os.path.join(target_dir, item_name)
                    if os.path.isdir(item_path):
                        shutil.rmtree(item_path)
                except Exception as e:
                    print(f"Failed to delete backup {item_path}: {e}")