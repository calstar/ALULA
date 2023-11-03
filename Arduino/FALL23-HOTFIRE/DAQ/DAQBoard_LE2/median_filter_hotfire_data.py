import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os

def median_filter(arr, BUFFER_SIZE):
    filtered_arr = np.array(arr)

    for i in range(0, len(arr)):
        filtered_arr[i] = np.median(arr[max(i - BUFFER_SIZE + 1, 0) : (i + 1)])

    return filtered_arr


data_file_path = os.path.join(os.path.dirname(__file__), "hotfire_serialplot_2023-04-16_test2.csv")

data = open(data_file_path).readlines()
data = np.array([row.split(',') for row in data if len(row.split(',')) == 15])

time_values = np.array(data[:, 1]).astype(np.float32)


fig, ax = plt.subplots(3, 3)
for i in range(2, 10):
    data_arr = data[:, i].astype(np.float32)
    ax[(i - 2) // 3][(i - 2) % 3].plot(time_values, data_arr)
    ax[(i - 2) // 3][(i - 2) % 3].plot(time_values, median_filter(data_arr, 500))

plt.show()
