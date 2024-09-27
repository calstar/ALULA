import serial
import time
import matplotlib.pyplot as plt
import IPython
from IPython import display
import numpy

# Open the serial connection (replace 'COM3' with the appropriate port for your system)
ser = serial.Serial('COM4', 115200)  # Adjust 'COM3' to the correct port for your ESP32
time.sleep(2)  # Wait for the serial connection to initialize
iteration = []
flow_rate = []
P1 = []
P2 = []
plt.ion()
t = [0]
# Open a text file for writing
with open('run2.txt', 'w') as file:
    try:
        while True:
            # Read a line from the serial port
            line = ser.readline().decode('utf-8').strip()
            # Write the line to the file
            file.write(line + '\n')
            line = line.split("\t")
            
            print(line)

            flow_rate.append(int(line[1]))
            P1.append(float(line[3]))
            P2.append(float(line[4]))

            display.clear_output(wait = True)
            display.display(plt.gcf)

            plt.clf()
            plt.xlabel("iteration")
            plt.ylim(-600, 600)

            plt.plot(t, flow_rate, label = "flow_rate (L/min)")
            plt.plot(t, P1, label = "P1 (psi)")
            plt.plot(t, P2, label = "P2 (psi)")
            plt.legend()
            plt.grid()

            plt.show(block = False)
            plt.pause(0.1)
            t.append(0.25 + t[-1])
    except KeyboardInterrupt:
        print("Data logging stopped")
    finally:
        ser.close()