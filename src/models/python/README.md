# YOLOv5 Tiled Image Processing Script
This Python script allows you to process an image using the YOLOv5 object detection model and split it into tiles for analysis.\
It's a convenient tool for object detection tasks on large images.

## Usage
1. **Prerequisites**:\
Ensure that you have Python installed on your system.
2. **Installation**:\
Run the following command to install the required packages:
	```bash
	pip install opencv-python ultralytics
	```
3. **Run the Script**:\
Execute the script using the following command:
	```bash
	python yolov5_tiled_predict.py
	```
4. **Follow On-Screen Prompts**:
	* Select the YOLO model file (with a `.pt` extension).
	* Choose the image file (supported formats: png, jpg, jpeg).
	* Specify the tile dimensions (must match the training dimensions of the model).
5. **Processing Tiles**:
	* The program will split the image into tiles and perform object detection on each tile.
	* Detected objects will be displayed with bounding boxes and confidence scores.
	* Close each tile's window to proceed to the next one.

## Notes
* If the file selection window does not appear, check if it opens in the background.
* The script is compatible with YOLOv5 models and requires a model file with a `.pt` extension.
* The tile dimensions must match the dimensions used to train the model.
* Performance statistics for each tile are provided.

## Dependencies
* OpenCV (`opencv-python`):\
Image processing library.
* Ultralytics (`ultralytics`):\
Library for YOLOv5 model.
