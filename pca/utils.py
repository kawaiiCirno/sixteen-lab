import csv
import numpy as np


def read_csv(filename):
    return np.genfromtxt(filename, dtype=np.float, delimiter=",")


def train_test_split(data, split_ratio=0.7):
    """Splits data into training and test set according to the split_ratio.

    Arguments:
        data: dataset as a numpy array
        split_ratio: fraction of dataset to split as training data (must be between 0 and 1)

    Returns:
        Training Data (size = split_ratio * size of original dataset)
        Test Data (size = (1 - split_ratio) * size of original dataset)
    """
    train_data, test_data = data[:int(split_ratio *
                                      len(data)), :], data[int(split_ratio *
                                                               len(data)):, :]

    return train_data, test_data
