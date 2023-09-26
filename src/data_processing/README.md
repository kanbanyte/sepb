
<!-- TOC ignore:true -->
# Data Processing Scripts
**Table of Contents**
<!-- TOC -->

* [calculate_training_ratio_roboflow.py](#calculate_training_ratio_roboflowpy)
* [convert_case.py](#convert_casepy)
* [copy_by_interval.ps1](#copy-by-intervalps1)
* [crop.py](#croppy)
* [image_processing.py](#image_processingpy)
* [random_crop.py](#random_croppy)
* [rename.ps1](#renameps1)
* [slice.py](#slicepy)

<!-- /TOC -->

## calculate_training_ratio_roboflow.py
Robowflow asks for training/test/validation ratio which is invalid after the augmentation process because the training set size is increased by a factor.
This script calculates the amount of images used for training, testing and validation accounting for extra images from the augmentation process.

## convert_case.py
Contains function that converts the bounding boxes of cases to position from 1 to 17, with 1 being at the bottom of the case rack.
This function requires the case image to be cropped such that the image bottom aligns with the bottom of the horizontal T-slot bar and the height of the image is around 514px.

## copy-by-interval.ps1
Copy files from a folder with a user-defined interval.\
Since the exported photos contain a lot of duplicates, copying them in an interval somewhat removes duplicate images.\
The user can specify the input and output folders.

## crop.py
Defines a crop box and applies it to all selected files.\
The following options are supported:
1. Select an image to define a crop box, then apply it to a set of selected images and save them to a folder.
2. Enter the crop box coordinates into the console, apply it to a set of images and save them to a folder.
3. Capture an image from the ZED camera and use it to define a crop box.

> **Note**:\
> The crop box is printed to the terminal, use it to manually change the program to use that crop box and apply it to images by batches.

## image_processing.py
Contains functions that processes images.\
Functionalities involve cropping, tiling and drawing rectangular crop boxes on images.

## random_crop.py
Creates a specified number of random crops from an image with the specified size.\
Primarily used to generates background images to diversify dataset for models that rely on static cropping.\
Without these images, the model can make many false positive predictions when used outside the cropped area within an image.

## rename.ps1
Rename all files in a folder into the format `<index>`.`<extension>`, with `<index>` being user defined.\
Since the output of the ZED Export program are images in the format `<left>/<right><index>.<extension>`, renaming them helps prevent duplicate names.
The renamed images will be copied into an output folder selected by the user.

## slice.py
Slice the images into a grid of 4 columns and 6 rows, corresponding to the structure of the 2 chip trays.\
This is to check how tight the crop box is so we can translate the concrete coordinates of bounding boxes into the position matrix easily.
