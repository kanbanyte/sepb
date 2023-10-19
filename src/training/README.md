
<!-- TOC ignore:true -->
# Training
**Table of Contents**
<!-- TOC -->

* [object_detection_training.ipynb](#object_detection_trainingipynb)
* [object_detection_training.py](#object_detection_trainingpy)
* [plot_ultralytics_result.ps1](#copy-by-intervalps1)

<!-- /TOC -->

## object_detection_training.ipynb
Notebook that downloads the dataset from Roboflow and train the YOLO model.\
The user is allowed to input various parameters such as epochs, image size, batch size, frozen layers, etc.\
If the notebook is run on Google Collab, the result folder of every training session can be downloaded as a zip file.

## object_detection_training.py
Python script designed to contain the same behaviour as `object_detection_training.ipynb`.\
The original intent was to convert the notebook to this script so it can be run on Swinburne Supercomputer if Google Collab does not offer enough hardware resources.\
However, as of 20/09/2023, the need to use the supercomputer has not arisen yet and the script might be outdated.

## plot_ultralytics_results.py
Training the YOLO model from Ultralytics will produce several images showing the model's metrics.\
However, YOLO does not support displaying overlays of different metrics, such as showing the training curve and validation curve in the same graph.\
This script is created to address this issue by plotting them based on the data retrieved from the `results.csv` file produced after every training session.