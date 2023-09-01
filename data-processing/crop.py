import subprocess
import importlib
import sys
import os
import tkinter as tk
from tkinter import filedialog
from PIL import Image
from pathlib import Path
import cv2

def select_output_folder():
    root = tk.Tk()
    root.withdraw()
    root.wm_attributes('-topmost', 1)
    return filedialog.askdirectory(parent=root, initialdir=os.getcwd(), title="Select output folder")

def filter_files_in_folder_with_extension(prompt, allowed_extensions):
    root = tk.Tk()
    root.withdraw()
    root.wm_attributes('-topmost', 1)
    file_paths = list(filedialog.askopenfilenames(parent=root, initialdir=os.getcwd(), title=prompt, filetypes=[("Allowed Files", f"*.{ext}") for ext in allowed_extensions]))
    return file_paths

def crop_image(input_image_path, output_image_path, crop_box):
    left, top, right, bottom = crop_box
    image = Image.open(input_image_path)
    cropped_image = image.crop((left, top, right, bottom))
    cropped_image.save(output_image_path)

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

def select_crop_coordinates(image_filename):
    top_left_corner = None
    bottom_right_corner = None
    original = None

    def draw_rectangle(event, x, y, flags, param):
        # this allows variables outisde the function scope to be accessible inside the function
        nonlocal top_left_corner, bottom_right_corner

        # mark the position where the user pressed the left button as the top left corner
        # and mark the position where the user releases that button as the bottom right corner
        if event == cv2.EVENT_LBUTTONDOWN:
            top_left_corner = [(x,y)]
        elif event == cv2.EVENT_LBUTTONUP:
            bottom_right_corner = [(x,y)]
            cv2.rectangle(image, top_left_corner[0], bottom_right_corner[0], (0,255,0), 2, 8)

    # Load the image
    image = cv2.imread(image_filename)

    # keep the original so we can reset later
    original = image.copy()

    window_name = "DRAW THE CROP BOX. PRESS 'R' TO RESET, PRESS 'f' TO FINISH OR QUIT"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, draw_rectangle)

    while True:
        cv2.imshow(window_name, image)
        key = cv2.waitKey(1)

        if key == ord('r'):
            image = original.copy()
        elif key == ord('f'):
            break;

    cv2.destroyAllWindows()

    if top_left_corner is not None and bottom_right_corner is not None:
        x1, y1 = top_left_corner[0]
        x2, y2 = bottom_right_corner[0]
        return x1, y1, x2, y2

    print("No crop box selected")
    exit()

def make_cropped_image_path(original_images, output_folder):
    new_paths = []
    for original_image in original_images:
        filename = os.path.basename(original_image)
        new_paths.append(os.path.join(output_folder, f"cropped.{filename}"))

    return new_paths

def main():
    extensions = ["jpg", "jpeg", "png"]
    template_image = select_file_with_extension(prompt="SELECT AN IMAGE TO DEFINE THE CROP BOX", allowed_extensions=extensions)
    crop_box = select_crop_coordinates(template_image)

    output_folder = select_output_folder()
    input_images = filter_files_in_folder_with_extension("SELECT IMAGES TO APPLY THE CROP ON", extensions)
    cropped_images = make_cropped_image_path(input_images, output_folder)

    for image_file, cropped_image in zip(input_images, cropped_images) :
        crop_image(image_file, cropped_image,  crop_box)

if __name__ == "__main__":
    main()
