import os
import tkinter as tk
from tkinter import filedialog
from PIL import Image
import cv2

def select_output_folder(root):
    """
    Displays a dialog box to select an output folder.

    Args:
        root (tk.Tk): The parent window or widget.

    Returns:
        str: The selected output folder path.
    """
    folder = filedialog.askdirectory(parent=root, initialdir=os.getcwd(), title="Select output folder")
    if folder is None:
        print("No output folder selected")
        exit()

    return folder

def select_images_to_crop(prompt, allowed_extensions, root):
    """
    Displays a dialog box to select images for cropping.

    Args:
        prompt (str): The prompt message for the dialog box.
        allowed_extensions (list): List of allowed image file extensions.
        root (tk.Tk): The parent window or widget.

    Returns:
        list: List of selected image file paths for cropping.
    """
    init_tkinter()
    file_paths = list(filedialog.askopenfilenames(parent=root, initialdir=os.getcwd(), title=prompt, filetypes=[("Allowed Files", f"*.{ext}") for ext in allowed_extensions]))
    return file_paths

def crop_image(input_image_path, output_image_path, crop_box):
    """
    Crops an image based on the specified crop box and saves the cropped image.

    Args:
        input_image_path (str): Path to the input image.
        output_image_path (str): Path to save the cropped image.
        crop_box (tuple): Tuple containing crop box coordinates (left, top, right, bottom).

    Returns:
        None
    """
    left, top, right, bottom = crop_box
    image = Image.open(input_image_path)
    cropped_image = image.crop((left, top, right, bottom))
    cropped_image.save(output_image_path)

def select_image_to_define_cropbox(prompt, allowed_extensions, root):
    """
    Selects an image file to define the crop box.

    Args:
        input_image_path (str): Path to the input image.
        output_image_path (str): Path to save the cropped image.
        crop_box (tuple): Tuple containing crop box coordinates (left, top, right, bottom).

    Returns:
        str: The selected image file
    """
    file_path = filedialog.askopenfilename(parent=root, initialdir=os.getcwd(), title=prompt, filetypes=[("Allowed Files", f"*.{ext}") for ext in allowed_extensions])

    if not file_path:
        print("No file selected")
        print("Exiting")
        exit()

    file_extension = os.path.splitext(file_path)[1][1:]
    if file_extension not in allowed_extensions:
        print(f"Selected file has an unsupported extension: {file_extension}")
        print("Exiting")
        exit()

    return file_path

def define_cropbox(image_filename):
    """
    Define cropbox on an image.

    Args:
        image_filename (str): Path to the input image.

    Returns:
        crop_box (tuple): Tuple containing crop box coordinates (left, top, right, bottom).
    """
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

    image = cv2.imread(image_filename)

    # keep the original so we can reset later
    original = image.copy()

    window_name = "DRAW THE CROP BOX. PRESS 'R' TO RESET, PRESS 'f' TO FINISH OR QUIT"

    # open a window and make sure the image scales down to the screen resolution
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    cv2.setMouseCallback(window_name, draw_rectangle)

    while True:
        cv2.imshow(window_name, image)

        # From OpenCV documentation:
        # "If you are using a 64-bit machine, you will have to modify k = cv2.waitKey(0) line as follows"
        key = cv2.waitKey(1) & 0xFF

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

def make_croppped_image_paths(original_images, output_folder):
    """
    Create file paths for cropped images based on the original image names and chosen output folder.

    Args:
        original_images (str): Paths to the original images.
        output_folder (str): Path to the output folder.

    Returns:
        list[str]: List containing new names for cropped images.
    """
    new_paths = []
    for original_image in original_images:
        filename = os.path.basename(original_image)
        new_paths.append(os.path.join(output_folder, f"cropped.{filename}"))

    return new_paths

# Initialize Tkinter and make sure the file dialog opens in the foreground
def init_tkinter():
    root = tk.Tk()
    root.withdraw()
    root.wm_attributes('-topmost', 1)
    return root

def main():
    root = init_tkinter()
    image_extensions = ["jpg", "jpeg", "png"]
    template_image = select_image_to_define_cropbox("SELECT AN IMAGE TO DEFINE THE CROP BOX", image_extensions, root)
    crop_box = define_cropbox(template_image)

    output_folder = select_output_folder(root)
    input_images = select_images_to_crop("SELECT IMAGES TO APPLY THE CROP ON", image_extensions, root)
    cropped_images = make_croppped_image_paths(input_images, output_folder)

    print(f"Applying {crop_box} crop to {len(input_images)} images")
    for image_file, cropped_image in zip(input_images, cropped_images):
        crop_image(image_file, cropped_image,  crop_box)

    print(f"Finished processing {len(input_images)} images")

if __name__ == "__main__":
    main()
