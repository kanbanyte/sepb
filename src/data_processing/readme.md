<!-- TOC ignore:true -->
# Data Processing Scripts
**Table of Contents**
<!-- TOC -->

* [crop.py](#croppy)
* [copy_by_interval.ps1](#copy-by-intervalps1)
* [rename.ps1](#renameps1)
* [slice.py](#slice.py)

<!-- /TOC -->

## crop.py
* Purpose:\
Define a crop box and apply it all selected files.
* Usage:
	1. Select an image to define the crop box on.
	2. Define the crop box on that image.\
	Press 'f' to finish or cancel.\
	Press 'r' to reset.
	3. Select image file(s) to apply that crop box on.
	4. Select output folder.
	5. Wait until finish.
* Required Packages:
	* Pillow `pip install pillow`
	* cv2 (OpenCV) `pip install opencv-python`
	* tkinter
* Note:
	* The crop box is printed to the terminal, use it to manually change the program to use that crop box and apply it to images by batches.

## copy-by-interval.ps1
* Purpose:\
Copy files from a folder with a user-defined interval.\
Since the exported photos contain a lot of duplicate, copying them in an interval somewhat removes duplicate images.
* Usage:
	1. Enter source folder.
	2. Enter target folder.
	3. Enter interval.
	4. Wait until finish.

## rename.ps1
* Purpose:\
Rename all files in a folder into the format `<index>`.`<extension>`, with `<index>` being user defined.\
Since the output of the ZED Export program are images in the format `<left>/<right><index>.<extension>`, renaming them helps prevent duplicate names.
* Usage:
	1. Enter the source folder.
	2. Enter the target folder.
	3. Enter starting index.

## slice.py
* Purpose:\
Slice the images into a grid of 4 columns and 6 rows, corresponding to the structure of the 2 chip trays. This is to check that later on we can translate the concrete coordinates of bounding boxes into the position matrix easily.
* Usage
	1. Select the file to slice
	2. View results in the current folder
