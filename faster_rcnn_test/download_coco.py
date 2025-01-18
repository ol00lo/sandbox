import requests
import zipfile
import io
import os
from tqdm import tqdm

urls = {
    "val2017": "http://images.cocodataset.org/zips/val2017.zip",
    "annotations": "http://images.cocodataset.org/annotations/annotations_trainval2017.zip"
}

image_folder = "val2017"
annotations_folder = "annotations"

def download_file(url, dest_folder):
    if not os.path.exists(dest_folder):
        os.makedirs(dest_folder)

    response = requests.get(url, stream=True)
    total_size = int(response.headers.get('content-length', 0))
    block_size = 1024  

    with open(os.path.join(dest_folder, url.split("/")[-1]), 'wb') as file, tqdm(
            desc=url.split("/")[-1],
            total=total_size,
            unit='iB',
            unit_scale=True,
            unit_divisor=1024,
    ) as bar:
        for data in response.iter_content(block_size):
            file.write(data)
            bar.update(len(data))

download_file(urls["val2017"], ".")
print("val2017 downloaded")
with zipfile.ZipFile("val2017.zip", 'r') as zip_ref:
    zip_ref.extractall(image_folder)

download_file(urls["annotations"], ".")
print("annotations downloaded")
with zipfile.ZipFile("annotations_trainval2017.zip", 'r') as zip_ref:
    zip_ref.extractall(annotations_folder)