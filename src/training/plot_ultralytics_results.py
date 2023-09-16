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

    # Figure showing the training and validation losses
    _, loss_axes = plt.subplots(3, 1, figsize=(10, 10))

    training_box_loss = df['         train/box_loss']
    validation_box_loss = df['           val/box_loss']
    loss_axes[0].plot(epochs, training_box_loss, label='Training Box Loss', marker=',', color='blue')
    loss_axes[0].plot(epochs, validation_box_loss, label='Validation Box Loss', marker=',', color='red')
    loss_axes[0].set_xlabel('Epochs')
    loss_axes[0].set_ylabel('Box Loss')
    loss_axes[0].set_title('Training and Validation Box Loss')
    loss_axes[0].legend()

    training_cls_loss = df['         train/cls_loss']
    validation_cls_loss = df['           val/cls_loss']
    loss_axes[1].plot(epochs, training_cls_loss, label='Training Classification Loss', marker=',', color='blue')
    loss_axes[1].plot(epochs, validation_cls_loss, label='Validation Classification Loss', marker=',', color='red')
    loss_axes[1].set_xlabel('Epochs')
    loss_axes[1].set_ylabel('Classification Loss')
    loss_axes[1].set_title('Training and Validation Classification Loss')
    loss_axes[1].legend()

    training_dfl_loss = df['         train/dfl_loss']
    validation_dfl_loss = df['           val/dfl_loss']
    loss_axes[2].plot(epochs, training_dfl_loss, label='Training DFL Loss', marker=',', color='blue')
    loss_axes[2].plot(epochs, validation_dfl_loss, label='Validation DFL Loss', marker=',', color='red')
    loss_axes[2].set_xlabel('Epochs')
    loss_axes[2].set_ylabel('DFL Loss')
    loss_axes[2].set_title('Training and Validation DFL Loss')
    loss_axes[2].legend()

    # Adjust spacing between subplots
    plt.tight_layout()

    # Figure containg precision and recall
    _, pr_axes = plt.subplots(2, 1, figsize=(10, 10))

    maAP50 = df['       metrics/mAP50(B)']
    mAP50_95 = df['    metrics/mAP50-95(B)']
    pr_axes[0].plot(epochs, maAP50, label='mAP50', marker=',', color='blue')
    pr_axes[0].plot(epochs, mAP50_95, label='mAP50-95', marker=',', color='red')
    pr_axes[0].set_xlabel('Epochs')
    pr_axes[0].set_ylabel('Mean Average Precisions Loss')
    pr_axes[0].set_title('Mean Average Precisions @50 and @50-95')
    pr_axes[0].legend()

    recall = df['      metrics/recall(B)']
    precision = df['   metrics/precision(B)']
    pr_axes[1].plot(epochs, recall, label='Recall', marker=',', color='blue')
    pr_axes[1].plot(epochs, precision, label='Precision', marker=',', color='red')
    pr_axes[1].set_xlabel('Epochs')
    pr_axes[1].set_ylabel('Percentage')
    pr_axes[1].set_title('Precision and Recall rates')
    pr_axes[1].legend()

    # Adjust spacing between subplots
    plt.tight_layout()

    plt.show()

if __name__ == "__main__":
	main()
