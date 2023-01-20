import time

import pandas as pd
import csv
import numpy as np
import serial
b = True

esp = serial.Serial(port = '/dev/cu.usbserial-0001',baudrate = 115200,timeout=.1) #change based on pot in arduino and baudrate

    # b = False
    # data = esp.readline()
    # decoded_bytes = data[0:len(data)-2].decode("utf-8")
    # if decoded_bytes == "Starting press-sequence":
    #     b = True
    #     break



# #mac pps would have to use port as "/dev/cu.usb0001" or whatever
# print("succesfully connected at port" )
# input = startPressSequence
while b:
    data = esp.readline()
    time.sleep(0.05)
    try:
        decoded_bytes = float(data[0:len(data)-2].decode("utf-8"))
        print(decoded_bytes)
        with open("res_data.csv","a") as f:
            writer = csv.writer(f,delimiter= ",")
            writer.writerow([time.time(),decoded_bytes])

    except:
        continue
