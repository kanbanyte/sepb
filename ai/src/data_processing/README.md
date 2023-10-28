
<!-- TOC ignore:true -->
# Data Processing APIs
**Table of Contents**
<!-- TOC -->

* [case_position.py](#case_positionpy)
* [image_processing.py](#image_processingpy)
* [tray_position.py](#tray_positionpy)
* [chip_position.py](#chip_positionpy)

<!-- /TOC -->

## case_position.py
Contains function that converts the bounding boxes of cases to position from 1 to 17, with 1 being at the bottom of the case rack.

**Important**
* This function requires the case image to be cropped such that the image bottom aligns with the bottom of the horizontal T-slot bar.
The height of the image is around 514px and this limit is not flexible.
* Before attempting to pick the case with the cobot, it is important to test the model and this code on every position on the rack.
The sample programs found in [src/samples/](../samples/README.md) can be helpful in this regard.

## image_processing.py
Contains functions that modifies images.
Functionalities involve cropping, drawing rectangular crop boxes on images with labels and displaying them.

## tray_position.py
Translate bounding boxes of tray into a cobot move.
Supported cobot moves include:
* Move tray 1 to assembly
* Move tray 2 to assembly
* Move tray 1 from assembly
* Move tray 2 from assembly
* Start loading items on tray 1
* Start loading items on tray 2
* Continue loading items on partially loaded tray 1
* Continue loading items on partially loaded tray 2

**Important**:
* The Tray Detection model does not detect individual items on a tray, it simply classifies trays into 3 categories: empty, partially full, and full.
The "Continue Loading Trays" move is intended to signal that the tray is partially full, which may or not may not be an error depending on the state of the cobot.
* The position conversion code assumes the following:
	* There are only 2 trays in 3 possible positions: Assembly, Tray 1 and Tray 2, with Assembly being on the left, Tray 1 and Tray 2 being on the right.
	* The image containing the tray should be roughly 600px in height and 700px in width,
	with Assembly tray on the left half, and Trays 1 and 2 vertically aligned in the right half.

## chip_position.py
Translate bounding boxes of chips into positions defined by the cobot.
These positions are numbered from 1 to 48, with 1 being the top left and 48 being the bottom right.
The positions increase by columns than rows.

**Important**
* The code requires the prediction to be made on a image with a height of around 230px and width of around 320px.
* The position is calculated using the center point of individual bounding boxes and checking them against a range of valid values.
The center point is used instead of the entire bounding box as the box can appear to be horizontally shifted when viewing a chip from an angle.
