import os, sys
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../util"))
from file_dialog import select_file_from_dialog, select_folder_from_dialog

import cv2
import random

def random_crop(image_path, crop_width, crop_height):
    """
    Tile an image with the specified grid size.

    Args:
        image_path (str): path to input image.
        crop_width (int): width of crop box.
        crop_height (int): height of crop box.

    Returns:
        np.array: cropped image.
    """
    image = cv2.imread(image_path)
    height = image.shape[0]
    width = image.shape[1]

    if height < crop_height:
        raise ValueError("Error: image height must be bigger than crop box height")

    if width < crop_width:
        raise ValueError("Error: image width must be bigger than crop box width")

    # Generate random coordinates for the crop box
    left = random.randint(0, width - crop_width)
    top = random.randint(0, height - crop_height)
    right = left + crop_width
    bottom = top + crop_height

    return image[top:bottom, left:right]

def main():
    image_extensions = ["jpg", "jpeg", "png"]
    original_image = select_file_from_dialog("Select image to generate random crops", image_extensions)
    if not original_image:
        raise ValueError("No image selected")

    count = int(input("Select number of crops to generate: "))

    output_folder = select_folder_from_dialog("Select output folder: ")
    if not output_folder:
        raise ValueError("No output folder selected")

    crop_height = int(input("Enter crop height: "))
    crop_width = int(input("Enter crop width: "))

    filename = os.path.basename(original_image)

    # apply random crop boxes to the selected image and save it in the selected folder
    for i in range(count):
        cropped_image = random_crop(original_image, crop_width, crop_height)
        cropped_image_path = os.path.join(output_folder, f"cropped-{i}-{filename}")
        cv2.imwrite(cropped_image_path, cropped_image)

if __name__ == "__main__":
    main()
