#!/usr/bin/env python3
import serial
import numpy as np
import signal
from threading import Event
from time import sleep
import sys

print(sys.argv[1])

exit_event = Event()
exit_event.set()

def int_handler(sig, num):
  exit_event.clear()
signal.signal(signal.SIGINT, int_handler)

def read_mag(port: serial.Serial) -> np.ndarray:
  mag_readings = port.readline().decode()[:-2].split(',')
  mag_readings = list(map(lambda e : float(e), mag_readings))
  return np.array(mag_readings).reshape(1,-1)

port = serial.Serial("/dev/cu.usbserial-AC00UOLH", baudrate=115200)
port.readline()

# data = read_mag(port)
data = ""
ctr = 0

while exit_event.is_set():
  # data = np.insert(data, data.shape[0], read_mag(port), axis=0)
  data += port.readline().decode()

  if not ctr % 100:
    print(f"{ctr} samples captured")
  ctr += 1

# np.save("magnetometer_data.npy", data)
with open(sys.argv[1], "w") as f:
  f.write(data)

port.close()

print("exiting")
