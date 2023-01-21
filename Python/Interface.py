import time

import pandas as pd
import csv
import numpy as np
import serial
b = True

esp = serial.Serial(port = '/dev/cu.usbserial-0001',baudrate = 115200,timeout=.1) #change based on pot in arduino and baudrate
name = "res_data5.csv"


while b:
    data = esp.readline()
    time.sleep(0.05)
    try:
        decoded_bytes = (data[0:len(data)-2].decode("utf-8"))
        values = decoded_bytes.split(",")
        with open(name,"a") as f:
            writer = csv.writer(f,delimiter= ",")
            guy = [time.time()]
            for i in range(len(values)):
                guy.append(values[i])
            writer.writerow(guy)

    except:
        continue
