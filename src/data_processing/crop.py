import os, sys
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../util"))
from cli_runner import install_packages
install_packages(["opencv-python"])

import cv2
from image_processing import crop_image
from file_dialog import select_folder_from_dialog
from file_dialog import select_files_from_dialog
from file_dialog import select_file_from_dialog

def select_output_folder():
    """
    Displays a dialog box to select an output folder.

    Args:

    Returns:
        str: The selected output folder path.
    """
    folder = select_folder_from_dialog("Select your output folder")
    if folder is None:
        print("No output folder selected")
        exit()

    return folder

def select_images_to_crop(prompt, allowed_extensions):
    """
    Displays a dialog box to select images for cropping.

    Args:
        prompt (str): The prompt message for the dialog box.
        allowed_extensions (list): List of allowed image file extensions.

    Returns:
        list[str]: List of selected image file paths for cropping.
    """

    return select_files_from_dialog(prompt, allowed_extensions)

def crop_image_and_save(input_image_path, output_image_path, crop_box):
    """
    Crops an image based on the specified crop box and saves the cropped image.

    Args:
        input_image_path (str): Path to the input image.
        output_image_path (str): Path to save the cropped image.
        crop_box (tuple): Tuple containing crop box coordinates (left, top, right, bottom).

    Returns:
        None
    """
    image = cv2.imread(input_image_path)
    cropped_image = crop_image(image, crop_box)
    cv2.imwrite(output_image_path, cropped_image)

def select_image_to_define_cropbox(prompt, allowed_extensions):
    """
    Selects an image file to define the crop box.

    Args:
        crop_box (tuple): Tuple containing crop box coordinates (left, top, right, bottom).

    Returns:
        str: The selected image file
    """
    file_path = select_file_from_dialog(prompt, allowed_extensions)

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

def define_crop_box_via_image(image_filename):
    """
    Define cropbox on an image.

    Args:
        image_filename (str): Path to the input image.

    Returns:
        x1, y1, x2, y2: Tuple containing crop box coordinates (left, top, right, bottom).
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

def make_cropped_image_paths(original_images, output_folder):
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

def get_crop_box(image_extensions=None):
    """
    Gets the crop box based on user choice. Two options are available: define the crop box on an image or entering its coordinates into the console.

    Args:
        image_extensions (list[str]): optional list containing image extensions without `.`

    Returns:
        x1, y1, x2, y2: Tuple containing crop box coordinates (left, top, right, bottom).
    """
    print(
    """
Select how a crop box is defined:
- 0: Define a crop box using a template image
- 1: Define a crop box by entering the xyxy coordinates (left-top-right-bottom) in the console
    """)
    choice = int(input("Enter your choice (0 or 1): "))
    if choice == 0:
        if image_extensions is None:
            image_extensions = ["jpg", "jpeg", "png"]
        template_image = select_image_to_define_cropbox("SELECT AN IMAGE TO DEFINE THE CROP BOX", image_extensions)
        return define_crop_box_via_image(template_image)
    elif choice == 1:
        x1 = int(input("Enter x1 coordinate (left): "))
        y1 = int(input("Enter y1 coordinate (top): "))
        x2 = int(input("Enter x2 coordinate (right): "))
        y2 = int(input("Enter y2 coordinate (bottom): "))

        if (x2 < x1):
            print("Error: x2 value (right) must be higher than x1 value (left)")
            exit(-1)

        if (y2 < y1):
            print("Error: y2 value (bottom) must be higher than y1 value (top)")
            exit(-1)

        return (x1, y1, x2, y2)

def main():

    image_extensions = ["jpg", "jpeg", "png"]
    crop_box = get_crop_box(image_extensions)

    output_folder = select_output_folder()
    print(f"Using output folder: {output_folder}")
    input_images = select_images_to_crop("SELECT IMAGES TO APPLY THE CROP ON", image_extensions)
    cropped_images = make_cropped_image_paths(input_images, output_folder)

    print(f"Applying {crop_box} crop to {len(input_images)} images")
    for image_file, cropped_image in zip(input_images, cropped_images):
        crop_image_and_save(image_file, cropped_image,  crop_box)

    print(f"Finished processing {len(input_images)} images")

if __name__ == "__main__":
    main()
