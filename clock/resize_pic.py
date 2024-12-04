import os
from PIL import Image

height = 512
width = 512

fold = 'clock_11'
input_folder = 'output\\' + fold
output_folder = 'new_output\\' + fold
os.makedirs(output_folder, exist_ok = True)

for filename in os.listdir(input_folder):
    if filename.lower().endswith('.jpg'):
        img_path = os.path.join(input_folder, filename)
        with Image.open(img_path) as img:
            w, h = img.size
            if w > width or h > height:
                img.thumbnail((width, height), Image.LANCZOS)
                output_path = os.path.join(output_folder, filename)
                img.save(output_path)
                print(f"Image {filename} resized and saved to {output_folder}.")
            else:
                print(f"Image {filename} is already smaller than 512x512, skipped.")

    else:
        print(f"File {filename} is not a image")