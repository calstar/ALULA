from PyQt5.QtWidgets import QApplication, QMainWindow, QGridLayout, QWidget, QPushButton, QVBoxLayout
import pyqtgraph as pg
from PyQt5.QtCore import QTimer
import sys
from threading import Thread
from queue import Queue
from serial import Serial
from collections import deque
import csv
from os.path import isfile
import time


data_len = 1500
num_plots = 11 #!!!!!!!!

BUFFER_SIZE = 100
write_buffer = []

deque_list = [deque(maxlen=data_len) for _ in range(num_plots + 3)]
x, PT_O1, PT_O2, PT_E1, PT_E2, PT_C1, LC_combined, LC1, LC2, LC3, TC1, TC2, TC3, TC4 = deque_list

plot_titles = ["PT_O1", "PT_O2", "PT_E1", "PT_E2", "PT_C1", "LC Combined", "LCs", "TC1", "TC2", "TC3", "TC4"]
button_names = ['Idle', 'Armed', 'Pressed', 'QD', 'Ignition', 'Hot Fire', 'Abort']

file_base = f"HOTFIRE_{time.strftime('%Y-%m-%d', time.gmtime())}"
file_ext = ".csv"
test_num = 1

while isfile(file_base + f"_test{test_num}" + file_ext):
    test_num += 1

filename = file_base + f"_test{test_num}" + file_ext

# for mac port_num = "/dev/cu.usbserial-0001"

port_num = "/dev/cu.usbserial-0001" # CHECK YOUR PORT !!!
esp32 = Serial(port=port_num, baudrate=115200)
# !!! IF NO NUMBERS PRINTED ON UR TERMINAL => PRESS "EN" ON THE ESP !!!


def collection():
    global values
    global COM_S 
    global DAQ_S

    with open(filename, "a", newline='') as f:
        writer = csv.writer(f, delimiter=",")

        while True:
            data = esp32.readline()
            try:
                decoded_bytes = data[:len(data)-2].decode("utf-8")
                values = decoded_bytes.split(" ")
                write_buffer.append(values)
              

                values = values[1:]


                if len(values) == 20:
                    
                    values = [safe_float(value) for value in values]
                    print(values)
                    x.append(float(values[0])/1000)
                    # S_Time.append(float(values[1])/1000)
                    # P_Time.append(float(values[2])/1000)
                    PT_O1.append(float(values[3]))
                    PT_O2.append(float(values[4]))
                    PT_E1.append(float(values[5]))
                    PT_E2.append(float(values[6]))
                    PT_C1.append(float(values[7]))

                    LC1.append(float(values[8]))
                    LC2.append(float(values[9]))
                    LC3.append(float(values[10]))
                    LC_combined.append(float(values[8]) + float(values[9]) + float(values[10]))

                    TC1.append(float(values[11]))
                    TC2.append(float(values[12]))
                    TC3.append(float(values[13]))
                    TC4.append(float(values[14]))

                    COM_S = values[17]
                    DAQ_S = values[18]

                if len(write_buffer) >= BUFFER_SIZE:
                        writer.writerows(write_buffer)
                        write_buffer.clear()

            except:
                continue
     

class LivePlotter(QMainWindow):
    def __init__(self, *args, **kwargs):
        super(LivePlotter, self).__init__(*args, **kwargs)

        self.centralWidget = QWidget()
        self.setCentralWidget(self.centralWidget)
        self.layout = QGridLayout(self.centralWidget)

        self.plotDataItems = []

        self.graphWidgets = [pg.PlotWidget(title=plot_titles[i]) for i in range(num_plots)]
        for i, graphWidget in enumerate(self.graphWidgets):
            self.layout.addWidget(graphWidget, i // 3, i % 3)

            if i != 6:  # For all other graphs
                plotDataItem = graphWidget.plot([], [])
                self.plotDataItems.append(plotDataItem)
            else:  # For graph 6, initialize and store three PlotDataItems
                self.plotDataItemsForGraph6 = [graphWidget.plot([], [], pen=pg.mkPen(color=(255, 0, 0))),
                                        graphWidget.plot([], [], pen=pg.mkPen(color=(0, 255, 0))),
                                        graphWidget.plot([], [], pen=pg.mkPen(color=(0, 0, 255)))]

        

        # # BUTTON STUFF
        # ###########################################################
        self.buttons = []  # Store button references if needed 
        self.buttonLayout = QVBoxLayout()

        for i, name in enumerate(button_names): 
            btn = QPushButton(name)

            btn.setStyleSheet("QPushButton {font-size: 14pt;}")
            #btn.clicked.connect(lambda _, name=name, number=i: self.handleButtonClick(name, number))
            btn.clicked.connect(lambda _, name=name, number=i: self.handleButtonClick(name, number))
            
            self.buttonLayout.addWidget(btn)  
            self.buttons.append(btn)
        self.buttonLayout.addStretch(1) 
        buttonWidget = QWidget()
        buttonWidget.setLayout(self.buttonLayout)
        self.layout.addWidget(buttonWidget, 0, 3, len(button_names) + 1, 1) 
        self.layout.setColumnStretch(0, 3) 
        self.layout.setColumnStretch(1, 3)
        self.layout.setColumnStretch(2, 3)
        self.layout.setColumnStretch(3, 1) 

        # # ########################################################### 
       
        self.timer = QTimer()
        self.timer.setInterval(300)  # ms
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

    def update_plot_data(self):

        try: 
            current_time = x[-1] if x else None

            if current_time is not None:
                min_time = current_time - 5

                while x and x[0] < min_time:
                    x.popleft()
                    for y_series in deque_list[1:]:  # Assuming self.y is a list of deques
                        y_series.popleft()

            self.setWindowTitle(f"Time: {current_time}    COM_state: {COM_S}   DAQ_state: {DAQ_S}")

            for i, plotDataItem in enumerate(self.plotDataItems):
                if i < 6:  # Update standard plots directly
                    plotDataItem.setData(list(x), list(deque_list[i + 1]))
                    self.graphWidgets[i].setTitle(f"{plot_titles[i]}: {deque_list[i + 1][-1]:.2f}")
            
            lc_data = [LC1, LC2, LC3]
            for plotDataItem, lc, title in zip(self.plotDataItemsForGraph6, lc_data, ["LC1", "LC2", "LC3"]):
                plotDataItem.setData(list(x), list(lc))

            for i in range(7, 11):
                self.plotDataItems[i-1].setData(list(x), list(deque_list[i + 3]))  # Adjust indices appropriately
                self.graphWidgets[i].setTitle(f"{plot_titles[i]}: {deque_list[i + 3][-1]:.2f}")
        except Exception as e:
        # Log the exception or handle it as needed
            print(f"Error updating plot data: {e}")

    def handleButtonClick(self, name, number):

        print(f"Button clicked: {name}")  
        esp32.write(str(number).encode()) 

def safe_float(value):
    try:
        return float(value) if value.lower() != "nan" else 0.0
    except ValueError:
        return 0.0


def main():
    t1 = Thread(target=collection)
    t1.start()

    app = QApplication(sys.argv)
    main_window = LivePlotter()
    main_window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
