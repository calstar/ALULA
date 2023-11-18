from serial import Serial
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from threading import Thread
import csv
from os.path import isfile
import time
import os


data_len = 500
num_plots = 11

BUFFER_SIZE = 100  
write_buffer = []

deque_list = [deque(maxlen=data_len) for _ in range(num_plots + 3)]
x, PT_O1, PT_O2, PT_E1, PT_E2, PT_C1, LC_combined, LC1, LC2, LC3, TC1, TC2, TC3, TC4 = deque_list

plot_titles = ["PT_O1", "PT_O2", "PT_E1", "PT_E2", "PT_C1", "LC Combined", "TC1", "TC2", "TC3", "TC4", "LCs"]

fig, axs = plt.subplots(4, 3)

axs_list = axs.flatten()
lines = [ax.plot([], [])[0] for ax in axs_list[:-1]]

# directory = os.getcwd() + "command_acquisition\FALL2023"
#  print(directory)
file_base = f"HOTFIRE_{time.strftime('%Y-%m-%d', time.gmtime())}"
file_ext = ".csv"
test_num = 1

while isfile(file_base + f"_test{test_num}" + file_ext):
    test_num += 1

filename = file_base + f"_test{test_num}" + file_ext

# for mac port_num = "/dev/cu.usbserial-0001"

port_num ="/dev/cu.usbserial-0001" # CHECK YOUR PORT !!!
esp32 = Serial(port=port_num, baudrate=115200)
# !!! IF NO NUMBERS PRINTED ON UR TERMINAL => PRESS "EN" ON THE ESP !!!

def collection():
    global values

    with open(filename, "a", newline='') as f:
        writer = csv.writer(f, delimiter=",")

        while True:
            data = esp32.readline()
            try:
                decoded_bytes = data[:len(data)-2].decode("utf-8")
                values = decoded_bytes.split(" ")
                print(values)

            
                write_buffer.append(values)

                if len(values) == 16:
                    x.append(float(values[0])/1000)
                    PT_O1.append(float(values[1]))
                    PT_O2.append(float(values[2]))
                    PT_E1.append(float(values[3]))
                    PT_E2.append(float(values[4]))
                    PT_C1.append(float(values[5]))
                    LC1.append(float(values[6]))
                    LC2.append(float(values[7]))
                    LC3.append(float(values[8]))
                    LC_combined.append(-0.31888 * (float(values[6]) + float(values[7]) + float(values[8])) + 52501.829)
                    TC1.append(float(values[9]))
                    TC2.append(float(values[10]))
                    TC3.append(float(values[11]))
                    TC4.append(float(values[12]))

                if len(write_buffer) >= BUFFER_SIZE:
                        writer.writerows(write_buffer)
                        write_buffer.clear()

            except:
                continue

t1 = Thread(target=collection)
t1.start()

def animate(i):
    ax = None
    
    if len(x) > 0:
        min_x = max(x[-1] - 5, 0)
        max_x = x[-1]

        fig.suptitle(f"TIME ELAPSED {max_x}s  COM_STATE: {values[13]}  DAQ_STATE: {values[14]}", fontsize=12)
        plt.tight_layout()

        for j, ax in enumerate(axs_list[:-1]):
            line = lines[j]
            if j < 6:  # For all plots up to and including LC_combined
                line.set_data(x, deque_list[j+1])
                title_data = deque_list[j+1][-1]
            elif j == 6:  # For TC1
                line.set_data(x, deque_list[10])  # TC1
                title_data = deque_list[10][-1]
            elif j == 7:  # For TC2
                line.set_data(x, deque_list[11])  # TC2
                title_data = deque_list[11][-1]
            elif j == 8:  # For TC3
                line.set_data(x, deque_list[12])  # TC2
                title_data = deque_list[12][-1]
            elif j == 9:  # For TC4
                line.set_data(x, deque_list[13])  # TC2
                title_data = deque_list[13][-1]

            ax.set_xlim(min_x, max_x)
            ax.set_title(f"{plot_titles[j]}: {title_data:.2f}", fontsize=12)
            ax.relim()
            ax.autoscale_view()

        ax = axs_list[-2]
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