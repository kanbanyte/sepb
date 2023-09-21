import cv2
import random

import sys, os; sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../"))
from util import select_file_from_dialog
from util import select_folder_from_dialog

def random_crop(image_path, crop_width, crop_height):
    # Read the image
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
    count = int(input("Select number of crops to generate: "))
    output_folder = select_folder_from_dialog("Select output folder: ")

    crop_height = int(input("Enter crop height: "))
    crop_width = int(input("Enter crop width: "))

    filename = os.path.basename(original_image)

    for i in range(count):
        cropped_image = random_crop(original_image, crop_width, crop_height)
        cropped_image_path = os.path.join(output_folder, f"cropped-{i}-{filename}")
        cv2.imwrite(cropped_image_path, cropped_image)

if __name__ == "__main__":
    main()
