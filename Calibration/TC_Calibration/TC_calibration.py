from serial import Serial
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import time
from os.path import isfile
import csv
import math

def read_serial():
    port_num = "/dev/cu.usbserial-0001"
    esp32 = Serial(port=port_num, baudrate=115200)

    try:
        while True:
            data = esp32.readline()
            try:
                decoded_bytes = data[:len(data)-2].decode("utf-8")
                str_data = decoded_bytes.split(" ")
                print("got it", str_data)

                if len(str_data) == instrument_count:
                   [i.append(float(x)) for i, x in zip(instrument_deques, str_data)]
                
            except:
                continue

    except KeyboardInterrupt:
        interrupt(instrument_deques) # control C

def interrupt(instrument_deques):
    input_reading = float(input("What is the pressure gauge reading? (Numbers only) \n"))
    Y.append(input_reading)

    [X[i].append(np.median(instrument_deques[i])) for i in range(instrument_count)]

    if len(Y) > 1:
        graphing(X, Y)
    else:
        with open(filename, "a", newline='') as f:
            writer = csv.writer(f, delimiter=",")
            row = []
            for j in range(instrument_count):
                row.extend([X[j][0], Y[0], 0, 0])
            writer.writerow(row)

def graphing(X, Y):
    num_cols = math.ceil(math.sqrt(instrument_count))
    num_rows = math.ceil(instrument_count / num_cols)

    fig, axs = plt.subplots(num_rows, num_cols, figsize=(12, 3* num_cols))

    val_row = []
    for j in range(instrument_count):
        row, col = divmod(j, num_cols)

        x = np.array(X[j])
        y = np.array(Y)
        # Find line of best fit
        a, b = np.polyfit(x, y, 1)

        axs[row, col].scatter(x, y, color='purple')

        axs[row, col].plot(x, a * x + b, color='steelblue', linestyle='--', linewidth=2,
                           label=f'y = {b:.6f} + {a:.6f}x')

        axs[row, col].set_title(f"Reading {j + 1}")
        axs[row, col].legend()

        val_row.extend([x[-1], y[-1], a, b])

    # Write data
    with open(filename, "a", newline='') as f:
        writer = csv.writer(f, delimiter=",")
        writer.writerow(val_row)

    # Remove any unused subplots
    for j in range(instrument_count, num_rows * num_cols):
        row, col = divmod(j, num_cols)
        fig.delaxes(axs[row][col])

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    global instrument_count
    instrument_count = int(input("Pls input the number of instruments for calibration (INT ONLY): "))

    data_point_num = 5

    instrument_deques = [deque(maxlen=data_point_num) for _ in range(instrument_count)]
    X = [[] for _ in range(instrument_count)]
    Y = []

    file_base = f"calibration_{time.strftime('%Y-%m-%d', time.gmtime())}"
    file_ext = ".csv"
    test_num = 1

    while isfile(file_base + f"_test{test_num}" + file_ext):
        test_num += 1

    filename = file_base + f"_test{test_num}" + file_ext

    with open(filename, "a", newline='') as f:
        writer = csv.writer(f, delimiter=",")

        row = []
        for j in range(instrument_count):
            row.extend([f"Reading {j + 1} X", f"Reading {j + 1} Y", "Slope (m)", "Intercept (c)"])
        
        writer.writerow(row)

    while True:
        read_serial()

        rerun = input("Would you like to run the PT calibration again? (y/n): ").strip().lower()
        if rerun != "y":
            break

