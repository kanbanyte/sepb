import numpy as np

def main():
    dataset_size = int(input("Enter value for dataset size: "))
    training_set_percentage = int(input("Enter the value for training set percentage (out of 100%): "))
    if training_set_percentage < 0 or training_set_percentage > 100:
        print("Training set percentage must be between 0 - 100%")
        exit(-1)

    training_set_augmentation_scale = int(input("Enter the value by which the training set will my scaled after the augmentation process: "))
    if training_set_augmentation_scale < 0:
        print("Training set augmentation scale must be positive")
        exit(-1)

    test_val_set_percentage = 100 - training_set_percentage

    coefficients = np.array([[1, 1], [test_val_set_percentage * training_set_augmentation_scale, training_set_percentage * -1]])
    constants = np.array([dataset_size, 0])
    training_size, _ = np.linalg.solve(coefficients, constants)

    rounded_training_size = round(training_size, 0)
    test_val_size = dataset_size - rounded_training_size
    test_size = round(test_val_size // 2, 0)
    validation_size = test_val_size - test_size

    print(f"Train set size: {rounded_training_size}")
    print(f"Test set size: {test_size}")
    print(f"Validation set size: {validation_size}")

if __name__ == "__main__":
    main()

