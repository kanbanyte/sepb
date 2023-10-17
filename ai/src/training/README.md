
<!-- TOC ignore:true -->
# Training
**Table of Contents**
<!-- TOC -->

* [object_detection_training.ipynb](#object_detection_trainingipynb)
* [plot_ultralytics_result.py](#plot_ultralytics_resultspy)

<!-- /TOC -->

## object_detection_training.ipynb
### Purpose:
Notebook that downloads the dataset from Roboflow and train the YOLO model.

### Usage
The user is allowed to input various parameters such as epochs, image size, batch size, frozen layers, etc.
If the notebook is run on Google Collab, the result folder of every training session can be downloaded as a zip file.

## plot_ultralytics_results.py
### Purpose:
Training the YOLO model from Ultralytics will produce several images showing the model's metrics.\
However, YOLO does not support displaying overlays of different metrics, such as showing the training curve and validation curve in the same graph.\
This script is created to address this issue by plotting them based on the data retrieved from the `results.csv` file produced after every training session.

### Usage
The user can select a `results.csv` file generated after a YOLO model is trained by Ultralytics.
If the `object_detection_training.ipynb` is used to train the model, the `results.csv` file is included in the output.
