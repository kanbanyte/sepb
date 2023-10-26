# AI Package

## Overview
The AI Package contains scripts and modules developed to interact with the ZED camera, run inference using detection models as well as process training data.
Image labelling is done in a web application known as Roboflow whereas training is done in a GPU-supported Google Collab session.

**Table of Contents**
<!-- TOC -->

* [Dependencies](#dependencies)
* [Installation](#installation)
* [Usage](#usage)
* [Common Issues](#common-issues)

## Dependencies
<!-- TOC ignore:true -->
### Hardware Requirements
* A computer running on an NVIDIA GPU with CUDA support.

<!-- TOC ignore:true -->
### Python Packages
* To ensure that the machine has packages to run every script in this directory, run the following command:
	```bash
	# replace path/to/requirements.txt
	python3 -m pip install -r path/to/requirements.txt
	```

<!-- TOC ignore:true -->
### ZED2 SDK
* Install the ZED2 SDK from [the official site](https://www.stereolabs.com/developers/release/).
	* Install the CUDA version suggested by the SDK and do not skip it.
	The installation continues if you choose to ignore CUDA but the code will not work.
* Install the Python API by running the `get_python_api.py` script:
	* On Windows, the script is in `C:\Program Files (x86)\ZED SDK\`
	* On Linux, the script is in `/usr/local/zed/`
	* See the [official documentation](https://www.stereolabs.com/docs/app-development/python/install/#installing-the-python-api) for more information.

## Installation
* Note: use `python` if you are running in Windows and `python3` if you are running in Linux.
This document exclusively uses the Linux version.
* To ensure you have the latest pip, run the command:
	```bash
	python3 -m pip install --upgrade pip
	```
* To install the package, navigate to the project containing `pyproject.toml`, remove the any existing `build` directory and run the below command.
This command should be run every time the package code is modified:
	```bash
	python3 -m pip install .
	```

## Usage

<!-- TOC ignore:true -->
### Directory Structure
The below list provides an overview of the directory structure.
See the `README.md` files within individual subdirectories for more details.
* [camera](src/camera/README.md): contains modules interacting with the ZED SDK Python API.
* [data_processing](src/data_processing/README.md): contains modules that process images and interact with the ZED2i camera.
* [models](src/models/README.md): contains modules related to a Python class representing a trained model.
* [samples](src/samples/README.md): contains sample programs that use trained models to detect objects and demonstrate usage of supported APIs.
* [scripts](src/scripts/README.md): containing runnable scripts that process training data.
These scripts are converted into executable files when the AI package is installed.
They are also used to process training data, balancing the dataset splits, defining crop boxes, etc.
* [training](src/training/README.md): contains notebooks and scripts that train a model and visualize its metrics.
Trained models and their settings are also included.
* [util](src/util/README.md): utility functions, mostly related to the file system.

<!-- TOC ignore:true -->
### Image Labelling
Training images are labelled and stored in a web application known as [Roboflow](https://app.roboflow.com/sepb).
Roboflow supports image labelling, image augmentation, image reprocessing,  dataset generation, and dataset balancing.
The dataset can be downloaded manually to your local machine or installed programmatically via Roboflow's APIs.

Generally, creating a dataset for a model involves the following steps:
1. Create a new project in your workspace and choose the model's purpose.
2. Upload raw image files.
3. Label images.
4. Apply image preprocessing, augmentation and dataset balancing settings.
5. Generate a version of the dataset.

The following articles from Roboflow can be helpful:
* [Getting Started with Roboflow](https://blog.roboflow.com/getting-started-with-roboflow/).
* [How to Train YOLOv5-Classification on a Custom Dataset](https://blog.roboflow.com/train-yolov5-classification-custom-data/).
* [How to Detect Small Objects: A Guide](https://blog.roboflow.com/detect-small-objects/).

<!-- TOC ignore:true -->
### Training
The model is created by retraining an existing YOLOv5 model developed by [Ultralytics](https://docs.ultralytics.com/).
Models are trained via a free Google Collab session running with an NVIDIA GPU.
You may choose to train the model elsewhere, as long as it has an NVIDIA GPU (since CUDA is required by Ultralytics YOLO) and at least 15GB of VRAM.
Training on images at larger sizes (above 800px) or training larger models (Large or Extra Large) consumes more VRAM.

The Jupyter notebook used for training relies on the dataset stored in Roboflow.
After a successful training session, the resulting model (.pt files) and its metrics can be downloaded to your local machine.
The model files (.pt files) are required by all code that uses the model so they must be saved if you intend to use the model for inference.
The Chip Detection model, Tray Detection model, and Case Detection model are included, see [this README](./src/training/README.md) for more information about dataset and training settings as well as recommended inference settings.

The training process involves the following steps:
1. Ensure you have the Roboflow private API key to download the dataset and know the project name.
	* Both can be accessed by selecting the dataset to be used, then click "Export", choose "Show download code", and copy the API key and the project name somewhere secure.
	* Robowflow may append random strings to the project name so it is important that you use the project name shown in the snippet.
	* The API key can also be accessed at [the workspace API settings](https://app.roboflow.com/sepb/settings/api).
2. Connect to a Google Collab session running on a T4 runtime.
3. Run the cells sequentially.
Do not advance to the next cell if the current one fails or has not completed.
The general flow of the notebook is:
	1. Download the dataset by supplying the API key and project name.
	2. Enter the settings to train the model and wait for the training to complete.
	3. View the model metrics.
	4. Optionally download the output folder which contains training metrics and model files.

## Common Issues
This section lists common issues observed during the development of the package and provides fixes or workarounds to avoid them.
Some of these solutions are merely suggestions and are not guaranteed to work.

<!-- TOC ignore:true -->
### AI Packages Not Updated:
**Symptoms:**\
The code in the AI package has been modified but the effects do not take place.

**Solution:**\
Ensure that the package with changed code has been reinstalled.
Keep in mind that any existing `build` folder must be deleted before reinstalling, otherwise the newly updated package is not installed, despite saying so in the terminal.

<!-- TOC ignore:true -->
### Camera Connection Failure:
**Symptoms:**\
The code that opens the camera fails with any error code.

**Solution:**\
Ensure that the camera cable is connected and no other application is using the camera.
Unplug the camera USB cable, wait for a few seconds and replug it.
If the above solutions do not work, try a different USB port on the machine.

<!-- TOC ignore:true -->
### Correct Detections But Incorrect Positions:
**Symptoms:**\
Detection models return correct bounding boxes but the numeric positions used by the cobot is not.
This can occur when the crop boxes are not sufficiently "tight" or correctly aligned.
The bounding box - to - position code has different criteria for different types of models and should be adhered to.
These requirements can be found in the [data_processing](src/data_processing/README.md) module.

**Solution:**\
Verify that the crop boxes are correctly aligned.
Keep in mind that any physical adjustments to the camera may alter its angle, leading to inaccuracies in existing crop boxes.
It is strongly recommended to assess the model's performance using sample scripts found in [samples](src/samples/README.md) before using it to guide the cobot.

<!-- TOC ignore:true -->
### Google Collab Connection Failure
**Symptoms:**\
Google Collab refuses to open a new session.

**Solution:**\
Unfortunately, using free sessions from Google Collab limits your usage time.
The time limit remains unclear, but it is suggested that Google Collab incrementally decrease your allocated time if you keep a session running for too long.
The solution is to either use the paid version, or limit your usage time as much as possible.

<!-- TOC ignore:true -->
### Images Not Displayed
**Symptoms:**\
The function `show_image()` in the [image_processing.py](src/data_processing/image_processing.py) module does not display images when called.
This is accompanied by warning messages that look like below in the terminal:\
`eog: symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE`

**Solution:**\
This is most likely caused by the `GTK_PATH` environment variable in VSCode's integrated terminals.
The following solutions can be used:
* Unset that variable, either by calling `unset GTK_PATH` or pasting this to the `settings.json` file of VSCode:
	```json
		"terminal.integrated.env.linux": {
			"GTK_PATH": ""
		}
	```
* Run the code in an external terminal.

Note that even when the images are successfully displayed, warning messages might appear in the terminal but can be ignored.
For more information, see [this StackOverflow thread](https://askubuntu.com/questions/1462295/ubuntu-22-04-both-eye-of-gnome-and-gimp-failing-with-undefined-symbol-error).

<!-- TOC ignore:true -->
### pyzed.sl Import Failure
**Symptoms:**\
Running code that uses the camera fails with the error
```bash
	import pyzed.sl as sl
ImportError: DLL load failed while importing sl: The specified module could not be found.
```

**Solution:**\
Ensure that the ZED SDK is installed correctly by checking the following steps:
* Your machine has CUDA which requires a NVIDIA GPU.
* The ZED SDK is installed.
* The Python API is installed via the script `get_python_api.py`.
See the [Dependencies](#dependencies) section for more details.

If all of the above steps fail, refer to these online threads or raise an issue with the ZED SDK maintainers themselves.
Note that none of these issues have been encountered during development after following the above steps:
* [GitHub Issue #1](https://github.com/stereolabs/zed-tensorflow/issues/10)
* [GitHub Issue #2](https://github.com/stereolabs/zed-python-api/issues/78)
* [GitHub Issue #3](https://github.com/stereolabs/zed-sdk/issues/358)
* [Stereo Labs Community Thread](https://community.stereolabs.com/t/importerror-no-module-named-pyzed-sl-as-sl/2467)
