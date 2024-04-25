from PyQt5.QtWidgets import QApplication, QMainWindow, QGridLayout, QWidget, QPushButton, QVBoxLayout
import pyqtgraph as pg # pip install pyqtgraph
from PyQt5.QtCore import QTimer
import sys #only needed for mac
from threading import Thread
from queue import Queue
from serial import Serial #pip install pyserial
from collections import deque
import csv
from os.path import isfile
import time


#RUNNING STEPS
#NAVIGATE TO FILE LOCATION IN TERMINAL
#PIP INSTALL STUF ABOVE
#  IF ERRORS: CHECK ENVIRONMENT (PYTHON VERSION, PATHS, ETC)
#EDIT PORT_NUM
#RUN COM CODE ON ARDUINO. CLOSE SERIAL MONITOR
#RUN THIS FILE
#PRESS BUTTONS/FLIP SWITHCHES TO CONTROL
#

data_len = 1500
num_plots = 6 #!!!!!!!!

BUFFER_SIZE = 100
write_buffer = []

deque_list = [deque(maxlen=data_len) for _ in range(num_plots + 1)]
x, PT_O1, PT_O2, PT_E1, PT_E2, PT_C1, PT_X = deque_list

plot_titles = ["PT_O1", "PT_O2", "PT_E1", "PT_E2", "PT_C1", "PT_X", "TC1", "TC2", "TC3", "TC4"]
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
    global FLIGHT_S
    global ETH_COMPLETE
    global OX_COMPLETE
    global ETH_VENT
    global OX_VENT
    global FLIGHT_Q_LENGTH
    global AUTO_ABORT
    global SD_CARD_STATUS


    with open(filename, "a", newline='') as f:
        writer = csv.writer(f, delimiter=",")

        while True:
            data = esp32.readline()
            try:
                decoded_bytes = data[:len(data)-2].decode("utf-8")
                values = decoded_bytes.split(" ")
                write_buffer.append(values)


                if len(values) == 31:

                    #  values = [safe_float(value) for value in values]
                    print(values)
                    x.append(float(values[0])/1000)
                    PT_O1.append(float(values[1]))
                    PT_O2.append(float(values[2]))
                    PT_E1.append(float(values[3]))
                    PT_E2.append(float(values[4]))
                    PT_C1.append(float(values[5]))
                    PT_X.append(float(values[6]))

                    # TC1.append(float(values[7]))
                    # TC2.append(float(values[8]))
                    # TC3.append(float(values[9]))
                    # TC4.append(float(values[10]))

                    COM_S = values[21]
                    DAQ_S = values[22]
                    FLIGHT_S = values[23]

                    ETH_COMPLETE = values[24]
                    OX_COMPLETE = values[25]

                    AUTO_ABORT = values[26]

                    ETH_VENT = values[27]
                    OX_VENT = values[28]

                    FLIGHT_Q_LENGTH = values[29]
                    SD_CARD_STATUS = values[30]


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
            btn.setStyleSheet("QPushButton {font-size: 25pt;}")
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

            self.setWindowTitle(f"Time: {current_time}    COM: {COM_S}   DAQ: {DAQ_S}   FLIGHT: {FLIGHT_S}  ETH_COMPLETE: {ETH_COMPLETE}  OX_COMPLETE: {OX_COMPLETE}   AUTO_ABORT: {AUTO_ABORT}   ETH_VENT: {ETH_VENT} OX_VENT: {OX_VENT}    Q_LENGTH: {FLIGHT_Q_LENGTH} SD_CARD_STATUS: {SD_CARD_STATUS}")


            for i, plotDataItem in enumerate(self.plotDataItems):
                if i < 10:  # Update standard plots directly
                    plotDataItem.setData(list(x), list(deque_list[i + 1]))
                    self.graphWidgets[i].setTitle(f"{plot_titles[i]}: {deque_list[i + 1][-1]:.2f}")


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
