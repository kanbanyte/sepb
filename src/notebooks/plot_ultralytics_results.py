import sys, os
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../util"))
from cli_runner import install_packages
from datetime import datetime
install_packages(["opencv-python", "ultralytics", "pandas", "matplotlib"])

from file_dialog import select_file_from_dialog
from file_reader import read_yaml

import pandas as pd
import matplotlib.pyplot as plt
import os

def main():
    results_csv = select_file_from_dialog("Select result CSV file", ["csv"])
    df = pd.read_csv(results_csv)

    # The csv file contains weird spaces before the labels.
    epochs = df['                  epoch']

    # Display training and validation loss curves for box
    training_box_loss = df['         train/box_loss']
    validation_box_loss = df['           val/box_loss']
    plt.figure(figsize=(10, 5))
    plt.plot(epochs, training_box_loss, label='Training Box Loss', marker='o', color='blue')
    plt.plot(epochs, validation_box_loss, label='Validation Box Loss', marker='o', color='red')
    plt.xlabel('Epochs')
    plt.ylabel('Box Loss')
    plt.title('Training and Validation Box Loss')
    plt.legend()

    # Display training and validation loss curves for classification
    training_cls_loss = df['         train/cls_loss']
    validation_cls_loss = df['           val/cls_loss']
    plt.figure(figsize=(10, 5))
    plt.plot(epochs, training_cls_loss, label='Training Classification Loss', marker='o', color='blue')
    plt.plot(epochs, validation_cls_loss, label='Validation Classification Loss', marker='o', color='red')
    plt.xlabel('Epochs')
    plt.ylabel('Classification Loss')
    plt.title('Training and Validation Classification Loss')
    plt.legend()

    # Display training and validation loss curves for DFL
    training_dfl_loss = df['         train/dfl_loss']
    validation_dfl_loss = df['           val/dfl_loss']
    plt.figure(figsize=(10, 5))
    plt.plot(epochs, training_dfl_loss, label='Training DFL Loss', marker='o', color='blue')
    plt.plot(epochs, validation_dfl_loss, label='Validation DFL Loss', marker='o', color='red')
    plt.xlabel('Epochs')
    plt.ylabel('DFL Loss')
    plt.title('Training and Validation DFL Loss')
    plt.legend()

    # Display training and validation loss curves for mean average precision
    maAP50 = df['       metrics/mAP50(B)']
    mAP50_95 = df['    metrics/mAP50-95(B)']
    plt.figure(figsize=(10, 5))
    plt.plot(epochs, maAP50, label='mAP50', marker='o', color='blue')
    plt.plot(epochs, mAP50_95, label='mAP50-95', marker='o', color='red')
    plt.xlabel('Epochs')
    plt.ylabel('Mean Average Precisions Loss')
    plt.title('Mean Average Precisions @50 and @50-95')
    plt.legend()

    # Display training and validation loss curves for precision and recall
    recall = df['      metrics/recall(B)']
    precision = df['   metrics/precision(B)']
    plt.figure(figsize=(10, 5))
    plt.plot(epochs, recall, label='Recall', marker='o', color='blue')
    plt.plot(epochs, precision, label='Precision', marker='o', color='red')
    plt.xlabel('Epochs')
    plt.ylabel('Percentage')
    plt.title('Precision and Recall rates')
    plt.legend()
    plt.show()

if __name__ == "__main__":
	main()
