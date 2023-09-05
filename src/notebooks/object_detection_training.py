import sys
sys.path.append("../util")
import os
import yaml
from IPython.display import Image, display
from cli_runner import install_packages
from cli_runner import run_command

# Check for NVIDIA GPU
# run_cli_command("nvidia-smi")

# Set up workspace directory

ROOT_DIR = os.getcwd()
print("Root directory is: " + ROOT_DIR)

# Install Roboflow and Dataset (with private API key)
print("Installing Roboflow package")
install_packages(["roboflow"])

from roboflow import Roboflow
import getpass

api_key = getpass.getpass("Enter your Roboflow project API key: ")
rf = Roboflow(api_key=api_key)

# Ensure that the following matches the intended dataset source:
# - Worspace name
# - Project name
# - Dataset version
# - Format
print(f"Downloading dataset from Roboflow")
project = rf.workspace("sepb").project("chips-cropped")
dataset = project.version(10).download("yolov5")

print("Fixing incorrect paths in 'data.yaml'")
dataset_yaml_path = os.path.join(dataset.location, "data.yaml")
with open(dataset_yaml_path) as data_yaml:
    doc = yaml.safe_load(data_yaml)
doc['train'] = "../train/images"
doc['val'] = "../valid/images"
with open(dataset_yaml_path, 'w') as data_yaml:
    yaml.dump(doc, data_yaml)

print("Finished fixing incorrect paths in 'data.yaml'")

# Install dependencies from Ultralytics in quiet mode

print("Installing Ultralytics dependencies")
install_packages(["ultralytics==8.0.159"])
import ultralytics
ultralytics.checks()
from ultralytics import YOLO

print("Cloning YOLOv5 from GitHub")
run_command("git clone https://github.com/ultralytics/yolov5 --quiet")
install_packages([], args=f"-r {ROOT_DIR}/yolov5/requirements.txt")

# According to the architecture of the model (.yaml file in yolov5/models), the backbone contains 10 layers
def freeze_backbone(trainer):
    model = trainer.model
    frozen_layer_count = int(input("Enter the number of layers to freeze"))
    if frozen_layer_count < 0 or frozen_layer_count > 10:
        print("Error: layer count must be between 0 and 10, inclusive")
        exit -1

    print(f"Freezing {frozen_layer_count} layers")
    freeze = [f'model.{x}.' for x in range(frozen_layer_count)]  # layers to freeze
    for k, v in model.named_parameters():
        v.requires_grad = True  # train all layers
        if any(x in k for x in freeze):
            print(f'Freezing {k}')
            v.requires_grad = False
    print(f"{frozen_layer_count} layers are freezed.")


# %cd {ROOT_DIR}

epochs_input = input(f"Enter the number of epochs (default: 30): ")
epochs = int(epochs_input) if epochs_input else 30

image_size_input = input(f"Enter the image size (default: 1000): ")
image_size = int(image_size_input) if image_size_input else 1000

model_choice = input("Enter the model choice (0 for small, 1 for medium, 2 for large, 3 for extra large): ")
if model_choice == "0":
    model_name = "yolov5su.pt"
elif model_choice == "1":
    model_name = "yolov5m.pt"
elif model_choice == "2":
    model_name = "yolov5l.pt"
elif model_choice == "3":
    model_name = "yolov5x.pt"
else:
    print("Invalid model choice. Please choose 0-3.")
    exit()

print(f"Training model {model_name} with {epochs} epochs")

# YOLO calculates the final learning rate as final_lr_factor * initial learning rate
final_lr_factor = 0.01

model = YOLO(model_name)

# The recommended way to freeze layers within the backbone is a callback https://github.com/ultralytics/ultralytics/issues/793#issuecomment-1510398080
model.add_callback("on_train_start", freeze_backbone)
model.train(data=dataset_yaml_path, epochs=epochs, imgsz=image_size, cache=True, lrf=final_lr_factor)

# Setup result output paths for subsequent cells
train_folders = [folder for folder in os.listdir(f'{ROOT_DIR}/runs/detect') if folder.startswith("train") and not folder.endswith(".zip")]

# Extract the indices from folder names and find the highest index
# Retraining will output results in new folders with the name format: "train<index>"
indices = [int(folder[len("train"):] if folder[len("train"):] else 0) for folder in train_folders]
highest_index = "" if max(indices) == 0 else max(indices)
result_folder_path = f'{ROOT_DIR}/runs/detect/train{highest_index}'

# Display the result summary

results_file_path = f'{result_folder_path}/results.png'
display(Image(filename=results_file_path, width=2000))

confusion_mat_file_path = f'{result_folder_path}/confusion_matrix.png'
display(Image(filename=confusion_mat_file_path, width=1600))

print(f"Displayed confusion matrix from {confusion_mat_file_path}")
print(f"Displayed result summary from {results_file_path}")
