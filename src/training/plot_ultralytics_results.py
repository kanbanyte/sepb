import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import sys, os
sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)),  "../"))
from util import select_file_from_dialog, select_file_from_dialog

def main():
    results_csv = select_file_from_dialog("Select result CSV file", ["csv"])
    print(f"Opening result file: {results_csv}")
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
    loss_axes[1].plot(epochs, np.asarray(training_cls_loss, float), label='Training Classification Loss', marker=',', color='blue')
    loss_axes[1].plot(epochs, np.asarray(validation_cls_loss, float), label='Validation Classification Loss', marker=',', color='red')
    loss_axes[1].set_xlabel('Epochs')
    loss_axes[1].set_ylabel('Classification Loss')
    loss_axes[1].set_title('Training and Validation Classification Loss')
    loss_axes[1].legend()

    training_dfl_loss = df['         train/dfl_loss']
    validation_dfl_loss = df['           val/dfl_loss']
    loss_axes[2].plot(epochs, np.asarray(training_dfl_loss, float), label='Training DFL Loss', marker=',', color='blue')
    loss_axes[2].plot(epochs, np.asarray(validation_dfl_loss, float), label='Validation DFL Loss', marker=',', color='red')
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
    pr_axes[0].plot(epochs, np.asarray(maAP50, float), label='mAP50', marker=',', color='blue')
    pr_axes[0].plot(epochs, np.asarray(mAP50_95, float), label='mAP50-95', marker=',', color='red')
    pr_axes[0].set_xlabel('Epochs')
    pr_axes[0].set_ylabel('Mean Average Precisions Loss')
    pr_axes[0].set_title('Mean Average Precisions @50 and @50-95')
    pr_axes[0].legend()

    recall = df['      metrics/recall(B)']
    precision = df['   metrics/precision(B)']
    pr_axes[1].plot(epochs,  np.asarray(recall, float), label='Recall', marker=',', color='blue')
    pr_axes[1].plot(epochs,  np.asarray(precision, float), label='Precision', marker=',', color='red')
    pr_axes[1].set_xlabel('Epochs')
    pr_axes[1].set_ylabel('Percentage')
    pr_axes[1].set_title('Precision and Recall rates')
    pr_axes[1].legend()

    # Adjust spacing between subplots
    plt.tight_layout()

    plt.show()

if __name__ == "__main__":
	main()
