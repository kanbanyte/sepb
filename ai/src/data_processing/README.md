
<!-- TOC ignore:true -->
# Data Processing APIs
**Table of Contents**
<!-- TOC -->

* [case_position.py](#case_positionpy)
* [image_processing.py](#image_processingpy)
* [random_crop.py](#random_croppy)
* [tray_position.py](#tray_positionpy)
* [chip_position.py](#chip_positionpy)

<!-- /TOC -->

## case_position.py
Contains function that converts the bounding boxes of cases to position from 1 to 17, with 1 being at the bottom of the case rack.\
This function requires the case image to be cropped such that the image bottom aligns with the bottom of the horizontal T-slot bar and
the height of the image is around 514px.

## image_processing.py
Contains functions that modifies images.\
Functionalities involve cropping, drawing rectangular crop boxes on images and displaying them.

## tray_position.py
Translate bounding boxes of tray into a cobot move.

## chip_position.py
Translate bounding boxes of chips into positions defined by the cobot.
These positions are numbered from 1 to 48, with 1 being the top left and 48 being the bottom right.
The positions increases by columns than rows.
Note that the code requires the prediction to be made on a tightly-cropped image.
