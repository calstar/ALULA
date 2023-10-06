from serial import Serial
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import time
from os.path import isfile
import csv
import shutil

process_array = []
data_length = 8

def sensor_calibrator():
    # Initial Reset
    # file_name = 'PTCal_test0'
    # folder_name = 'PTCal_testfold'
    # test_device = 'PT '

    data_point_num = 3



    # data_labels = [f'{test_device}{i}' for i in range(1, data_length + 1)]
    # data_labels.append(f'{test_device}Readings')

    port_num = "COM14"
    esp32 = Serial(port=port_num, baudrate=115200)

    # Open serial port and read data
    raw_data = []
    i = 1

    try:
        while True:
            data = esp32.readline()
            try:
                decoded_bytes = data[:len(data)-2].decode("utf-8")
                str_data = decoded_bytes.split(" ")
                print("got it")

                # with open(filename, "a", newline='') as f:
                #     writer = csv.writer(f, delimiter= ",")
                #     writer.writerow(str_data)
            except:
                continue

            if len(str_data) == data_length:
                raw_data.append([float(x) for x in str_data])

            if len(raw_data) > data_point_num:
                raw_data.pop(0)


    except KeyboardInterrupt:
        # User interrupted the process, now clean up
        clean_me_up(raw_data, data_length, esp32)

def clean_me_up(raw_data, data_length, s):
    global process_array

    reading = float(input("What is the pressure gauge reading? (Numbers only) \n"))



    # calculate mean values
    data_array = np.array(raw_data)
    mean_array = np.mean(data_array, axis=0)


    mean_vals = []

    [mean_vals.append([i, reading]) for i in mean_array]

    process_array.append(mean_vals)
    print(process_array)

    if len(process_array) > 1:
        data_processing_graphing(process_array)

def data_processing_graphing(array):
    file_base = f"testing_calibration_{time.strftime('%Y-%m-%d', time.gmtime())}"
    file_ext = ".csv"
    test_num = 1

    while isfile(file_base + f"_test{test_num}" + file_ext):
        test_num += 1

    filename = file_base + f"_test{test_num}" + file_ext



    fig, axs = plt.subplots(3, 3, figsize=(12, 8))

    for j in range(data_length):
        # Extract the j-th data set
        row, col = divmod(j, 3)

        X = [reading[j][0] for reading in array]
        Y = [reading[j][1] for reading in array]

        axs[row, col].plot(X, Y, 'o')

        trend = np.polyfit(X, Y, 1)
        trendpoly = np.poly1d(trend)
        axs[row, col].plot(X, trendpoly(X), label=f"y = {trend[0]:.2f}x + {trend[1]:.2f}")

        axs[row, col].set_title(f"Sensor {j + 1}")
        axs[row, col].legend()

        with open(filename, "a", newline='') as f:
            writer = csv.writer(f, delimiter=",")
            # Write the header
            writer.writerow([f"Sensor {j + 1} X", f"Sensor {j + 1} Y", "Slope (m)", "Intercept (c)"])
            # Write data
            for x_val, y_val in zip(X, Y):
                writer.writerow([x_val, y_val, trend[0], trend[1]])

    # Remove any unused subplots
    for j in range(data_length, 9):
        row, col = divmod(j, 3)
        fig.delaxes(axs[row][col])

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    while True:
        sensor_calibrator()
        rerun = input("Would you like to run the sensor calibration again? (y/n): ").strip().lower()
        time.sleep(10)
        if rerun != "y":
            break
