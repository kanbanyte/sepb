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

BOX_THICKNESS = 2
GREEN_RGB = (0, 255, 0)
FONT_THICKNESS = 2

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
    num_cols = get_positive_int("Select column count: ", 4)

    tiled_images = tile_image(image, num_rows, num_cols)

    for tile_index, tile in enumerate(tiled_images):
        print(f"Checking tile {tile_index}")
        results = model(tile, verbose=False)

        # TODO(HUY): why is there only one object in `results`
        for result_index, result in enumerate(results):
            # This is the built-in function that plots the bounding boxes
            # However, the full label-confidence text is too long and obscures parts of the image
            # cv2.imshow(f"Tile #{result_index} (close window to continue)", result.plot())

            if result.boxes is None or result.boxes.xyxy.numel() == 0:
                print(f"No object detected in tile {tile_index} (close image to continue)")
                continue

            if result.boxes is not None and result.boxes.xyxy.numel() != 0:
                x1_tensor = result.boxes.xyxy[:, 0]
                y1_tensor = result.boxes.xyxy[:, 1]
                x2_tensor = result.boxes.xyxy[:, 2]
                y2_tensor = result.boxes.xyxy[:, 3]

            detected_obj_count = x1_tensor.size()
            tile_result_data = zip(result.boxes.conf, x1_tensor, y1_tensor, x2_tensor, y2_tensor)
            for obj_index, (conf, x1, y1, x2, y2) in enumerate(tile_result_data):

                # OpenCV only works with integers
                x1_int = int(x1)
                y1_int = int(y1)
                x2_int = int(x2)
                y2_int = int(y2)

                cv2.rectangle(tile, (x1_int, y1_int), (x2_int, y2_int), GREEN_RGB, BOX_THICKNESS)
                cv2.putText(tile, f'{conf * 100:.2f}', (x1_int, y1_int - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, GREEN_RGB, FONT_THICKNESS)
                print(f"Object #{obj_index}/{detected_obj_count} - Tile #{tile_index}")
                print("Performance in ms:")
                for key, value in result.speed.items():
                    print(f"\t{key}: {value}")
                print(f"Confidence: {conf}")
                print(f"Bounding Box: "
                    f"(x1={x1:.2f}, y1={y1:.2f}), "
                    f"(x2={x2:.2f}, y2={y2:.2f})")
                print()

        cv2.imshow(f'Tile #{tile_index} with Bounding Boxes', tile)
        cv2.waitKey(0)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
