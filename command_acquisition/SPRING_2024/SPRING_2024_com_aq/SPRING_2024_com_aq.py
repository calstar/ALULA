from PyQt5.QtWidgets import QApplication, QMainWindow, QGridLayout, QWidget, QPushButton, QVBoxLayout
import pyqtgraph as pg
from PyQt5.QtCore import QTimer
import sys
from threading import Thread
from queue import Queue
import serial
from collections import deque


class SerialThread(Thread):
    def __init__(self, queue, port='/dev/cu.usbserial-0001', baudrate=115200):
        Thread.__init__(self)
        self.queue = queue
        self.port = port
        self.baudrate = baudrate

        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baudrate
        self.ser.timeout = 1
        self.running = True

    def run(self):
        try:
            self.ser.open()
        except serial.SerialException as e:
            print(f"Could not open serial port {self.ser.port}: {e}")
            return

        while self.running:
            data = self.ser.readline()
            try: 
                line = data[:len(data)-2].decode("utf-8").split(" ")
                #print(line)
                if len(line) >= 16:  
                    x_val = float(line[0]) /1000 
                    data_vals = [float(val) for val in line[3:]]  
                    self.queue.put((x_val, data_vals))
            except:
                continue

    def stop(self):
        self.running = False
        self.ser.close()

class LivePlotter(QMainWindow):
    def __init__(self, queue, serial_thread, *args, **kwargs):
        super(LivePlotter, self).__init__(*args, **kwargs)
        self.serial_thread = serial_thread

        self.centralWidget = QWidget()
        self.setCentralWidget(self.centralWidget)
        self.layout = QGridLayout()
        self.centralWidget.setLayout(self.layout)

        self.num_plots = 10  # Number of separate graphs
        self.graphWidgets = [pg.PlotWidget() for _ in range(self.num_plots)]

        self.dq_length = 1000

        self.x = deque([0 for _ in range(self.dq_length)], maxlen=self.dq_length)
        self.y = [deque([0 for _ in range(self.dq_length)], maxlen=self.dq_length) for _ in range(18)] 
        self.data_lines = []

        self.titles = ["PT_O1", "PT_O2", "PT_E1", "PT_E2", "PT_C1", 
                       "LCs", "TC1", "TC2", "TC3", "TC4"]

        # Initialize 
        #########################
        for i, graphWidget in enumerate(self.graphWidgets):
            self.layout.addWidget(graphWidget, i // 3, i % 3)  # Arrange plots in a 3x3 grid
            graphWidget.setTitle(f"{self.titles[i]}: Initializing...")  
            if i != 5:  # For all but the 5th graph
                data_line = graphWidget.plot([0], [0])
                self.data_lines.append([data_line])  
        
            else:  # Special handling for the 5th graph with 3 lines LC!
                self.data_lines.append([
                    graphWidget.plot([0], [0], pen=pg.mkPen(color=(255, 0, 0))),  # Red line
                    graphWidget.plot([0], [0], pen=pg.mkPen(color=(0, 255, 0))),  # Green line
                    graphWidget.plot([0], [0], pen=pg.mkPen(color=(0, 0, 255))),   # Blue line
                    graphWidget.plot([0], [0], pen=pg.mkPen(color=(255, 255, 0)))  # Yellow line for the sum
                ])

        # Add buttons
        self.buttons = []  # Store button references if needed
        self.button_names = ['Idle', 'Armed', 'Pressed', 'QD', 'Ignition', 'Hot Fire', 'Abort']  


        self.buttonLayout = QVBoxLayout()

        for i, name in enumerate(self.button_names):  # Start enumeration at 0
            btn = QPushButton(name)
            btn.setMinimumSize(120, 40)
            btn.setStyleSheet("QPushButton {font-size: 14pt;}")
            btn.clicked.connect(lambda _, name=name, number=i: self.handleButtonClick(name, number))
            self.buttonLayout.addWidget(btn)  # Add button to the QVBoxLayout
            self.buttons.append(btn)

        self.buttonLayout.addStretch(1)  # Add stretch to push all buttons up and add space at the bottom

        # Create a widget to set the layout
        buttonWidget = QWidget()
        buttonWidget.setLayout(self.buttonLayout)

        # Adjust the layout placement for the button widget
        # Assuming you still want it in the same grid position, replace the earlier button adding part with this:
        self.layout.addWidget(buttonWidget, 0, 3, len(self.button_names) + 1, 1)  # Span multiple rows to fit all buttons

        # Adjust grid layout column stretch to ensure buttons don't take too much horizontal space
        self.layout.setColumnStretch(0, 3)  # Assuming you want the plots to take more space
        self.layout.setColumnStretch(1, 3)
        self.layout.setColumnStretch(2, 3)
        self.layout.setColumnStretch(3, 1)  # Less space for buttons


        self.queue = queue
        self.timer = QTimer()
        self.timer.setInterval(50)  # ms
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def update_plot_data(self):
        current_time = None  # Track the most recent timestamp
        while not self.queue.empty():
            x_val, values = self.queue.get()
            if current_time is None or x_val > current_time:
                current_time = x_val  # Update the most recent timestamp
            # Append new data
            self.x.append(x_val)
            for i in range(16):
                self.y[i].append(values[i])

            # Filter data to keep only the last 10 seconds
            if current_time is not None:
                min_time = current_time - 10  # Calculate the threshold time
                while self.x and self.x[0] < min_time:  # Remove old data points
                    self.x.popleft()
                    for i in range(16):
                        self.y[i].popleft()
        
            self.setWindowTitle(f"Time: {self.x[-1]:.2f}    COM_state: {self.y[14][-1]:.2f}   DAQ_state: {self.y[15][-1]:.2f}")

            # Update the plot data
            for i in range(self.num_plots):  
                if i == 5: 
                    sum_LCs = [sum(values) for values in zip(self.y[i], self.y[i+1], self.y[i+2])]
                    for j in range(3):  # Update each of the original 3 lines
                        self.data_lines[i][j].setData(self.x, list(self.y[i+j]))
                        self.data_lines[i][3].setData(self.x, sum_LCs)  # Update the sum line
                    self.graphWidgets[i].setTitle(f"{self.titles[i]}: {sum_LCs[-1]:.2f}") 
                elif i > 5:
                    self.data_lines[i][0].setData(self.x, list(self.y[i + 2]))
                    self.graphWidgets[i].setTitle(f"{self.titles[i]}: {self.y[i+2][-1]:.2f}")
                else:
                    self.data_lines[i][0].setData(self.x, list(self.y[i]))
                    self.graphWidgets[i].setTitle(f"{self.titles[i]}: {self.y[i][-1]:.2f}")
    
    def handleButtonClick(self, name, number):
        print(f"{name}")  # Correct the print statement to display the button name
        if self.serial_thread.ser.isOpen():
            self.serial_thread.ser.write(str(number).encode())  # Use the serial_thread's ser attribute


def main():
    serial_data_queue = Queue()
    serial_thread = SerialThread(serial_data_queue)
    serial_thread.start()

    app = QApplication(sys.argv)
    main_window = LivePlotter(serial_data_queue, serial_thread) 
    main_window.show()

    try:
        sys.exit(app.exec_())
    finally:
        serial_thread.stop()
        serial_thread.join()

if __name__ == '__main__':
    main()
