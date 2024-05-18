from PyQt5.QtWidgets import QApplication, QMainWindow, QGridLayout, QWidget, QPushButton, QVBoxLayout, QLineEdit
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

pt_deques = [PT_O1, PT_O2, PT_E1, PT_E2, PT_C1, PT_X]

plot_titles = ["PT_O1", "PT_O2", "PT_E1", "PT_E2", "PT_C1", "PT_X", "TC1", "TC2", "TC3", "TC4"]
button_names = ['Idle', 'Armed', 'Pressed', 'QD', 'Ignition', 'Hot Fire', 'Abort']
pt_names = ['PT_O1', 'PT_O2', 'PT_E1', 'PT_E2', 'PT_C1', 'PT_X']
pt_offsets = [0, 0, 0, 0, 0, 0]

file_base = f"FLIGHTtest_{time.strftime('%Y-%m-%d', time.gmtime())}"
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
    # global SD_CARD_STATUS
    global pt_offsets


    with open(filename, "a", newline='') as f:
        writer = csv.writer(f, delimiter=",")

        while True:
            data = esp32.readline()
            try:
                decoded_bytes = data[:len(data)-2].decode("utf-8")
                values = decoded_bytes.split(" ")
                write_buffer.append(values)


                if len(values) == 28:

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

                    COM_S = values[13]
                    DAQ_S = values[14]
                    FLIGHT_S = values[15]

                    ETH_COMPLETE = values[16]
                    OX_COMPLETE = values[17]

                    AUTO_ABORT = values[18]

                    ETH_VENT = values[19]
                    OX_VENT = values[20]

                    FLIGHT_Q_LENGTH = values[21]
                    # SD_CARD_STATUS = values[30]

                    pt_offsets = values[22:28].copy()

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
            self.layout.addWidget(graphWidget, i // 3 + 1, i % 3)

            if i != 6:  # For all other graphs
                plotDataItem = graphWidget.plot([], [])
                self.plotDataItems.append(plotDataItem)
            else:  # For graph 6, initialize and store three PlotDataItems
                self.plotDataItemsForGraph6 = [graphWidget.plot([], [], pen=pg.mkPen(color=(255, 0, 0))),
                                        graphWidget.plot([], [], pen=pg.mkPen(color=(0, 255, 0))),
                                        graphWidget.plot([], [], pen=pg.mkPen(color=(0, 0, 255)))]
                

        # Store initial button styles
        self.initial_button_styles = {}


        # # BUTTON STUFF
        # ###########################################################
        self.buttons = []  # Store button references if needed
        self.offsetTextboxes = [] # Store textbox references if needed
        self.offsetButtons = [] # Store button references if needed
        self.zeroOffsetButtons = []

        self.buttonLayout = QVBoxLayout()
        self.offsetButtonLayout = QVBoxLayout()
        self.offsetTextLayout = QVBoxLayout()
        self.zeroOffsetButtonLayout = QVBoxLayout()

        self.oxCompleteLayout = QVBoxLayout()
        self.ethCompleteLayout = QVBoxLayout()
        self.ventLayout = QVBoxLayout()

        # Setup buttons
        # Setup buttons
        for i, name in enumerate(button_names):
            btn = QPushButton(name)
            btn.setStyleSheet("QPushButton {font-size: 50pt;}")
            self.initial_button_styles[btn] = btn.styleSheet()
            btn.clicked.connect(lambda _, name=name, number=i, btn=btn: self.handleButtonClick(name, number, btn))
            self.buttonLayout.addWidget(btn)
            self.buttons.append(btn)


        # # ###########################################################
        # Setup PT offsets
        # Setup buttons
        for i, pt_name in enumerate(pt_names):
            btn = QPushButton("Update offset: " + pt_name + f" ({pt_offsets[i]})")
            btn.setStyleSheet("QPushButton {font-size: 10pt;}")
            btn.clicked.connect(lambda _, name=pt_name, number=i: self.ptOffsetButtonClick(name, number))
            self.offsetButtonLayout.addWidget(btn)
            self.offsetButtons.append(btn)

        # Setup textboxes
        for pt_name in enumerate(pt_names):
            textbox = QLineEdit()
            textbox.setStyleSheet("QLineEdit {font-size: 15pt;}")
            self.offsetTextLayout.addWidget(textbox)
            self.offsetTextboxes.append(textbox)

        for i, pt_name in enumerate(pt_names):
            btn = QPushButton("ZERO offset")
            btn.setStyleSheet("QPushButton {font-size: 10pt;}")
            btn.clicked.connect(lambda _, name=pt_name, number=i: self.ptZeroOffsetButtonClick(name, number))
            self.zeroOffsetButtonLayout.addWidget(btn)
            self.zeroOffsetButtons.append(btn)

        self.buttonLayout.addStretch(1)
        buttonWidget = QWidget()
        buttonWidget.setLayout(self.buttonLayout)
        self.layout.addWidget(buttonWidget, 0, 3, len(button_names) + 1, 1)
        self.layout.setColumnStretch(0, 3)
        self.layout.setColumnStretch(1, 3)
        self.layout.setColumnStretch(2, 3)
        self.layout.setColumnStretch(3, 1)

        offsetButtonWidget = QWidget()
        offsetButtonWidget.setLayout(self.offsetButtonLayout)
        self.layout.addWidget(offsetButtonWidget, 3, 0, len(pt_names), 1)

        offsetTextWidget = QWidget()
        offsetTextWidget.setLayout(self.offsetTextLayout)
        self.layout.addWidget(offsetTextWidget, 3, 1, len(pt_names), 1)

        zeroOffsetTextWidget = QWidget()
        zeroOffsetTextWidget.setLayout(self.zeroOffsetButtonLayout)
        self.layout.addWidget(zeroOffsetTextWidget, 3, 2, len(pt_names), 1)

        # LED indicator
        self.oxCompleteIndicator = QPushButton("Ox\nComplete")
        self.ethCompleteIndicator = QPushButton("Eth\nComplete")

        self.abortIndicator = QPushButton("ABORT")
        self.ethVentIndicator = QPushButton("Ox\nvent")
        self.oxVentIndicator = QPushButton("Eth\nvent")

        self.oxCompleteLayout.addWidget(self.oxCompleteIndicator)
        self.ethCompleteLayout.addWidget(self.ethCompleteIndicator)

        self.ventLayout.addWidget(self.abortIndicator)
        self.ventLayout.addWidget(self.oxVentIndicator)
        self.ventLayout.addWidget(self.ethVentIndicator)

        oxCompleteWidget = QWidget()
        oxCompleteWidget.setLayout(self.oxCompleteLayout)
        ethCompleteWidget = QWidget()
        ethCompleteWidget.setLayout(self.ethCompleteLayout)

        ventWidget = QWidget()
        ventWidget.setLayout(self.ventLayout)

        self.layout.addWidget(oxCompleteWidget, 0, 0, 1, 1)
        self.layout.addWidget(ethCompleteWidget, 0, 2, 1, 1)
        self.layout.addWidget(ventWidget, 6, 3, 1, 1)

        # Timer
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

            self.setWindowTitle(f"Time: {current_time}    COM: {COM_S}   DAQ: {DAQ_S}   FLIGHT: {FLIGHT_S}  ETH_COMPLETE: {ETH_COMPLETE}  OX_COMPLETE: {OX_COMPLETE}   AUTO_ABORT: {AUTO_ABORT}   ETH_VENT: {ETH_VENT} OX_VENT: {OX_VENT}    Q_LENGTH: {FLIGHT_Q_LENGTH}")

            for i, plotDataItem in enumerate(self.plotDataItems):
                if i < 10:  # Update standard plots directly
                    plotDataItem.setData(list(x), list(deque_list[i + 1]))
                    self.graphWidgets[i].setTitle(f"{plot_titles[i]}: {deque_list[i + 1][-1]:.2f}")

            for i, offset in enumerate(pt_offsets):
                self.offsetButtons[i].setText("Update offset: " + pt_names[i] + f" ({pt_offsets[i]})")
        
            # Update indicator LED colors
            if OX_COMPLETE == "True":
                self.oxCompleteIndicator.setStyleSheet("background-color: green")
            else:
                self.oxCompleteIndicator.setStyleSheet("background-color: red")
            if ETH_COMPLETE == "True":
                self.ethCompleteIndicator.setStyleSheet("background-color: green")
            else:
                self.ethCompleteIndicator.setStyleSheet("background-color: red")
            if AUTO_ABORT == "True":
                self.abortIndicator.setStyleSheet("background-color: red")
                if OX_VENT == "True":
                    self.oxVentIndicator.setStyleSheet("background-color: green")
                else:
                    self.oxVentIndicator.setStyleSheet("background-color: red")
                if ETH_VENT == "True":
                    self.ethVentIndicator.setStyleSheet("background-color: green")
                else:
                    self.ethVentIndicator.setStyleSheet("background-color: red")
            else:
                self.abortIndicator.setStyleSheet("background-color: gray")
                self.oxVentIndicator.setStyleSheet("background-color: gray")
                self.ethVentIndicator.setStyleSheet("background-color: gray")

            self.updateButtonStyles()

        except Exception as e:
        # Log the exception or handle it as needed
            print(f"Error updating plot data: {e}")

    def handleButtonClick(self, name, number, btn):

        print(f"Button clicked: {name}")
        esp32.write(("s" + str(number)).encode())
        #btn.setStyleSheet("background-color: yellow; font-size: 50pt;")

    def updateButtonStyles(self):
        for i, btn in enumerate(self.buttons):
            if i == int(COM_S) == int(FLIGHT_S):  # Highlight the button corresponding to COM_S value
                btn.setStyleSheet("background-color: gray; font-size: 50pt;")
            else:
                btn.setStyleSheet(self.initial_button_styles[btn])  # Revert other buttons to original style


    def ptOffsetButtonClick(self, name, id):
        print(f"Button clicked: {name}")
        print("Serial write: ", "o" + str(id) + str(self.offsetTextboxes[id].text()))
        esp32.write(("o" + str(id) + str(self.offsetTextboxes[id].text())).encode())

    def ptZeroOffsetButtonClick(self, name, id):
        try:
            print(f"Zeroing offset for {id}")
            new_offset = float(pt_offsets[id]) - float(pt_deques[id][-1])
            print(f"New offset is {new_offset}")
            esp32.write(("o" + str(id) + str(new_offset)).encode())
        except Exception as e:
            print(e)
            pass

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