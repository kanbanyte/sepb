import os, sys

sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../util"))
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../camera"))

import cv2
from image_processing import crop_image
from file_dialog import select_folder_from_dialog
from file_dialog import select_files_from_dialog
from file_dialog import select_file_from_dialog
from camera_capture import capture_image
from camera_capture import open_camera
from file_reader import read_yaml

IMAGE_EXTENSIONS = ["jpg", "jpeg", "png"]

def select_output_folder():
    """
    Displays a dialog box to select an output folder.

    Args: None

    Returns:
        str: The selected output folder path.
    """
    folder = select_folder_from_dialog("Select your output folder")
    if not folder:
        raise ValueError("No output folder selected")

    return folder

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

def draw_crop_box_on_image(image):
    """
    Define crop box on an image.

    Args:
        image (np.array): image to draw crop box on.

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
            top_left_corner = [(x, y)]
        elif event == cv2.EVENT_LBUTTONUP:
            bottom_right_corner = [(x, y)]
            cv2.rectangle(image, top_left_corner[0], bottom_right_corner[0], (0,255,0), 2, 8)


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

    raise ValueError("No crop box selected")

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

def define_crop_box_with_image():
    """
    Open an image file and define the crop box on that image.

    Args: None

    Returns:
        (int,int,int,int): Tuple containing crop box coordinates in the left-top-right-bottom format.
    """
    template_image = select_file_from_dialog("SELECT AN IMAGE TO DEFINE THE CROP BOX", IMAGE_EXTENSIONS)
    image = cv2.imread(template_image)

    return draw_crop_box_on_image(image)

def define_crop_box_with_console():
    """
    Define the crop box by entering its coordinates to the terminal.

    Args: None

    Returns:
        (int,int,int,int): Tuple containing crop box coordinates in the left-top-right-bottom format.
    """
    x1 = int(input("Enter x1 coordinate (left): "))
    y1 = int(input("Enter y1 coordinate (top): "))
    x2 = int(input("Enter x2 coordinate (right): "))
    y2 = int(input("Enter y2 coordinate (bottom): "))

    if (x2 < x1):
        raise ValueError("Error: x2 value (right) must be higher than x1 value (left)")

    if (y2 < y1):
        raise ValueError("Error: y2 value (bottom) must be higher than y1 value (top)")

    return (x1, y1, x2, y2)

def define_crop_box_with_camera():
    """
    Capture an image from the ZED camera and define the crop box on that image.

    Args: None

    Returns:
        (int,int,int,int): Tuple containing crop box coordinates in the left-top-right-bottom format.
    """
    config_file = select_file_from_dialog("SELECT CAMERA CONFIGURATION FILE", ["yaml"])
    if not config_file:
        raise ValueError("Configuration file path is empty")

    print(f"Reading camera configuration file '{config_file}'")
    config = read_yaml(config_file)
    camera = open_camera(config.get('camera'))
    image = capture_image(camera)
    camera.close()
    return draw_crop_box_on_image(image)

def apply_crop_and_save(crop_box):
    """
    Apply the crop box to selected images and save them in a specified folder.

    Args:
        crop_box(int,int,int,int): Tuple containing crop box coordinates in the left-top-right-bottom format.

    Returns: None
    """
    output_folder = select_output_folder()
    if not output_folder:
        raise ValueError("Output folder is empty")

    print(f"Using output folder: {output_folder}")
    input_images = select_files_from_dialog("SELECT IMAGES TO APPLY THE CROP ON", IMAGE_EXTENSIONS)
    cropped_images = make_cropped_image_paths(input_images, output_folder)

    print(f"Applying {crop_box} crop to {len(input_images)} images")
    for image_file, cropped_image in zip(input_images, cropped_images):
        crop_image_and_save(image_file, cropped_image,  crop_box)

    print(f"Finished processing {len(input_images)} images")

def main():

    print(
    """
Select how a crop box is defined:
- 0: Define a crop box using a template image and apply it to selected images
- 1: Define a crop box by entering the xyxy coordinates (left-top-right-bottom) in the console and apply it to selected images
- 2: Define a crop box using an image captured from the ZED camera
    """)
    choice = int(input("Enter your choice (0, 1 or 2): "))
    if choice == 0:
        apply_crop_and_save(define_crop_box_with_image())
    elif choice == 1:
        apply_crop_and_save(define_crop_box_with_console())
    elif choice == 2:
        print(f"Defined crop box: {define_crop_box_with_camera()}")
    else:
        raise ValueError(f"Invalid choice")

if __name__ == "__main__":
    main()
