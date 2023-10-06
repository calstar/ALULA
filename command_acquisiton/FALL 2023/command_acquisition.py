from serial import Serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from threading import Thread
import csv
from os.path import isfile
import time
import os

data_len = 500
num_plots = 9

deque_list = [deque(maxlen=data_len) for _ in range(num_plots + 3)]
x, PT_O1, PT_O2, PT_E1, PT_E2, PT_C1, LC_combined, LC1, LC2, LC3, TC1, TC2 = deque_list

plot_titles = ["PT_O1", "PT_O2", "PT_E1", "PT_E2", "PT_C1", "LC Combined", "TC1", "TC2", "LCs"]

fig, axs = plt.subplots(3, 3)

axs_list = axs.flatten()
lines = [ax.plot([], [])[0] for ax in axs_list[:-1]]

directory = "/Users/ivysim/Desktop/LE2/command_acquisiton"
file_base = directory + f"testing_{time.strftime('%Y-%m-%d', time.gmtime())}"
file_ext = ".csv"
test_num = 1

while isfile(file_base + f"_test{test_num}" + file_ext):
    test_num += 1

filename = file_base + f"_test{test_num}" + file_ext

port_num = "/dev/cu.usbserial-0001"
esp32 = Serial(port=port_num, baudrate=115200)

def collection():
    global values

    while True:
        data = esp32.readline()
        try:
            decoded_bytes = data[:len(data)-2].decode("utf-8")
            values = decoded_bytes.split(" ")

            with open(filename, "a", newline='') as f:
                writer = csv.writer(f, delimiter=",")
                writer.writerow(values)

            if len(values) == 14:
                x.append(float(values[0])/1000)
                PT_O1.append(float(values[1]))
                PT_O2.append(float(values[2]))
                PT_E1.append(float(values[3]))
                PT_E2.append(float(values[4]))
                PT_C1.append(float(values[5]))
                LC1.append(float(values[6]))
                LC2.append(float(values[7]))
                LC3.append(float(values[8]))
                LC_combined.append(float(values[6]) + float(values[7]) + float(values[8]))
                TC1.append(float(values[9]))
                TC2.append(float(values[10]))

        except:
            continue

t1 = Thread(target=collection)
t1.start()

def animate(i):
    if len(x) > 0:
        min_x = max(x[-1] - 5, 0)
        max_x = x[-1]

        fig.suptitle(f"TIME ELAPSED {max_x}s  COM_STATE: {values[11]}  DAQ_STATE: {values[12]}", fontsize=12)
        plt.tight_layout()

        for j, ax in enumerate(axs_list[:-1]):
            if j < 6:  # For all plots up to and including LC_combined
                line = lines[j]
                line.set_data(x, deque_list[j+1])
                title_data = deque_list[j+1][-1]
            elif j == 6:  # For TC1
                line = lines[j]
                line.set_data(x, deque_list[10])  # TC1
                title_data = deque_list[10][-1]
            elif j == 7:  # For TC2
                line = lines[j]
                line.set_data(x, deque_list[11])  # TC2
                title_data = deque_list[11][-1]
            ax.set_xlim(min_x, max_x)
            ax.set_title(f"{plot_titles[j]}: {title_data:.2f}", fontsize=12)
            ax.relim()
            ax.autoscale_view()

        ax = axs_list[-1]
        ax.clear()
        ax.plot(x, LC1, label="LC1", color='blue')
        ax.plot(x, LC2, label="LC2", color='red')
        ax.plot(x, LC3, label="LC3", color='green')
        ax.set_title(plot_titles[-1], fontsize=12)
        ax.set_xlim(min_x, max_x)
        ax.legend(loc='upper left', fontsize=8)
        ax.relim()
        ax.autoscale_view()

    return lines + [ax]

ani = FuncAnimation(fig, animate, interval=10, cache_frame_data=False)
plt.tight_layout()
plt.show()
