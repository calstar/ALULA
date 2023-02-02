import csv
from os.path import isfile
from serial import Serial
import time

# Port num of esp32 depends on OS and usb slot. Check the Arduino IDE to verify port num on computer. For Mac, use "/dev/cu.usbserial-0001".
port_num = "COM14"
esp32 = Serial(port=port_num, baudrate = 115200, timeout=0.1)

# Create a filename in the format: res_data_{year}-{month}-{day}_test{num}.
file_base = f"res_data_{time.strftime('%Y-%m-%d', time.gmtime())}"
file_ext = ".csv"
test_num = 1
while isfile(file_base + f"_test{test_num}" + file_ext):
    test_num += 1
filename = file_base + f"_test{test_num}" + file_ext

while True:
    time.sleep(0.05)
    data = esp32.readline()
    try:
        decoded_bytes = data[:len(data)-2].decode("utf-8")
        values = decoded_bytes.split(",")
        print(values)
        with open(filename, "a", newline='') as f:
            writer = csv.writer(f, delimiter= ",")
            time_ms = time.time_ns() // (10**6)
            millisecs = str(time_ms % (10**3)).zfill(3)
            writer.writerow([time.strftime("%H:%M:%S", time.gmtime(time_ms // (10**3))) + ":" + millisecs] + values)
    except:
        continue