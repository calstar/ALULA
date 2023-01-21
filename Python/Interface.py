import csv
from serial import Serial
import time
from os.path import isfile

# Port num of esp32 depends on OS and usb slot. Check the Arduino IDE to verify port num on computer. For Mac, use "/dev/cu.usbserial-0001".
port_num = "COM3"
esp32 = Serial(port=port_num, baudrate = 115200, timeout=0.1)

# Create a filename in the format: res_data_{year}_{month}_{day}_test{count}.
file_base = f"res_data_{time.strftime('%Y_%m_%d', time.gmtime())}"
file_ext = ".csv"
count = 1
while isfile(file_base + f"_test{count}" + file_ext):
    count += 1
filename = file_base + f"_test({count})" + file_ext

    # b = False
    # data = esp.readline()
    # decoded_bytes = data[0:len(data)-2].decode("utf-8")
    # if decoded_bytes == "Starting press-sequence":
    #     b = True
    #     break
# print("succesfully connected at port" )
# input = startPressSequence

while True:
    time.sleep(0.05)
    data = esp32.readline()
    try:
        decoded_bytes = float(data[:len(data)-2].decode("utf-8"))
        print(decoded_bytes)
        with open(filename, "a") as f:
            writer = csv.writer(f, delimiter= ",")
            writer.writerow([time.time(), decoded_bytes])
    except:
        continue
