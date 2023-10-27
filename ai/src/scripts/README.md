
<!-- TOC ignore:true -->
# Training Data Processing Scripts
**Table of Contents**
<!-- TOC -->

* [dataset_balancer.py](#dataset_balancerpy)
* [copy_interval.py](#copy_intervalpy)
* [define_crop.py](#define_croppy)
* [random_crop.py](#random_croppy)

<!-- /TOC -->

## dataset_balancer.py
<!-- TOC ignore:true -->
### Purpose
Robowflow asks for training/test/validation ratio which is invalid after the augmentation process because the training set size is increased by a factor (2 or 3 when using a free tier).
This script calculates the amount of images used for training, testing and validation accounting for extra images from the augmentation process.

<!-- TOC ignore:true -->
### Usage
The user is asked to enter the dataset size, desired training set ratio after augmentation and
the factor by which the training set will be multiplied by the augmentation process in the terminal.
The program prints the recommended size for the train set, test set, and validation set to meet the desired ratio.

## copy_interval.py
<!-- TOC ignore:true -->
### Purpose
Copy files from a folder with a user-defined interval.\
This script is useful if you collect data by taking a video and export images (frames) from it.
The resulting image set contains many duplicates which can be trimmed down by selecting images at an interval.

<!-- TOC ignore:true -->
### Usage
The user is asked to select the source folder, destination folder and the interval by which an image from the source folder is copied to the destination one.

## define_crop.py
<!-- TOC ignore:true -->
### Purpose
Defines a crop box and applies it to all selected files.
Since the AI models rely on image that is cropped to a specific region of interest, cropping images are required for both training and evaluation.

<!-- TOC ignore:true -->
### Usage
The user will be asked to define a crop box and apply it to select images.
The crop box coordinates are printed in the terminal.
The following options are supported:
1. Select an image to define a crop box, then apply it to a set of selected images and save them to a folder.
2. Enter the crop box coordinates into the console, apply it to a set of images and save them to a folder.
3. Capture an image from the ZED camera and use it to define a crop box, then apply it to a set of images and save them to a folder.

## random_crop.py
<!-- TOC ignore:true -->
### Purpose
Creates a specified number of random crops from an image with the specified dimensions.

Primarily used for generating background images to diversify datasets for models relying on static cropping.
Without these additional images, the model may produce numerous false positive predictions when applied on an image outside the cropped area.
Including these extra background images enhances the models' accuracy and resilience when used on images that were trained with static cropping.

<!-- TOC ignore:true -->
### Usage
The user will be asked to supply the source image, enter the coordinates of the crop box, the number of images to generate, and the destination folder.

<!-- TOC ignore:true -->
## Notes
* If the file selection window does not appear, check if it opens in the background.
* User prompts for file dialogues are provided in the file selection window name.
