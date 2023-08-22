"""
Usage:
1. Run the script using the following command:
   python yolov5_tiled_predict.py
2. Follow the on-screen prompts to select model and image files, as well as set tile dimensions.
3. The program will process the image tiles using the YOLO model and display the results.
4. Close the image window to proceed to the next tile.

Note:
- Make sure you have Python installed on your system.
- The following packages will be installed if not available: opencv-python, ultralytics
- You will be prompted to select a YOLO model file (with a .pt extension) and an image file (png, jpg, jpeg). If the file selection window does not appear, it might be opened in the background.
- The tile dimensions must match the dimensions used to train the model.
"""

import subprocess
import importlib
import sys
import os
import cv2
import tkinter as tk
from tkinter import filedialog
from ultralytics import YOLO

def install_packages(packages_to_install):
    for package in packages_to_install:
        if importlib.util.find_spec(package) is None:
            print(f"Installing required package in quiet mode: {package}")
            subprocess.call(["pip", "install", package, "--quiet"])
        else:
            print(f"{package} is already installed.")

def tile_image(image, num_rows, num_cols):
    tile_height = int(image.shape[0] / num_rows)
    tile_width = int(image.shape[1] / num_cols)
    tiled_images = []

    for row in range(num_rows):
        for col in range(num_cols):
            y_start = row * tile_height
            y_end = y_start + tile_height
            x_start = col * tile_width
            x_end = x_start + tile_width

            tile = image[y_start:y_end, x_start:x_end]
            tiled_images.append(tile)

    return tiled_images

def select_file_with_extension(prompt, allowed_extensions):
    root = tk.Tk()
    root.withdraw()
    root.wm_attributes('-topmost', 1)
    file_path = filedialog.askopenfilename(parent=root, initialdir=os.getcwd(), title=prompt, filetypes=[("Allowed Files", f"*.{ext}") for ext in allowed_extensions])

    if not file_path:
        print("No file selected")
        print("Exiting")
        sys.exit()

    file_extension = os.path.splitext(file_path)[1][1:]
    if file_extension not in allowed_extensions:
        print(f"Selected file has an unsupported extension: {file_extension}")
        print("Exiting")
        sys.exit()

    return file_path


def get_positive_int(prompt, default_value):
    try:
        user_input = input(prompt)
        if (not user_input):
            print(f"No value entered, using default value: {default_value}")
            return default_value

        value = int(user_input)
        if value <= 0:
            print("Input must be a positive integer.")
            sys.exit()
        else:
            return value
    except ValueError:
        print("Input must be a positive integer.")
        sys.exit()

def main():
    install_packages(["opencv-python", "ultralytics"])

    model_file = select_file_with_extension("Select model file", ["pt"])
    model = YOLO(model_file)

    image_file = select_file_with_extension("Select model file", ["png", "jpg", "jpeg"])
    image = cv2.imread(image_file)

    print("Select tile dimensions (must be the same as the dimension used to train the model)")
    num_rows = get_positive_int("Select row count: ", 3)
    num_cols = get_positive_int("Select column: ", 4)

    tiled_images = tile_image(image, num_rows, num_cols)

    for i, tile in enumerate(tiled_images):
        print(f"Checking tile {i}")
        results = model(tile, verbose=False)

        # TODO(HUY): why is there only one object in `results`
        for result_index, result in enumerate(results):
            cv2.imshow(f"Tile #{result_index} (close window to continue)", result.plot())
            if result.boxes is None or result.boxes.xyxy.numel() == 0:
                print("No object detected in tile")
                continue

            if result.boxes is not None and result.boxes.xyxy.numel() != 0:
                x1_tensor = result.boxes.xyxy[:, 0]
                y1_tensor = result.boxes.xyxy[:, 1]
                x2_tensor = result.boxes.xyxy[:, 2]
                y2_tensor = result.boxes.xyxy[:, 3]
                for i in range(len(result.boxes.xyxy)):
                    print(f"Object #{i}")
                    print("Performance in ms:")
                    for key, value in result.speed.items():
                        print(f"    {key}: {value}")
                    print(f"Confidence: {result.boxes.conf[i] * 100:.2f}%")
                    print(f"Bounding Box: "
                        f"(x1={x1_tensor[i]:.2f}, y1={y1_tensor[i]:.2f}), "
                        f"(x2={x2_tensor[i]:.2f}, y2={y2_tensor[i]:.2f})")
                    print(f"\n")

        cv2.waitKey(0)
        print("\n")

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
