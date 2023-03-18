#!/usr/bin/env python3
import serial
import struct
import numpy as np
import signal
from threading import Event

exit_event = Event()
exit_event.set()

def int_handler(sig, num):
  exit_event.clear()
signal.signal(signal.SIGINT, int_handler)

def read_tof(port: serial.Serial) -> np.ndarray:
  buf = port.read(64 * 2 + 1)[:-1]
  if len(buf) == 128:
    return np.array(struct.unpack("<64H", buf))
  else:
    print(buf)
    return np.array([])

port = serial.Serial("/dev/cu.usbserial-AC00UOLH", baudrate=115200)
print(port.readline())
print(port.readline())

data = read_tof(port)
ctr = 0

print("started collecting")
while exit_event.is_set():
  sample = read_tof(port)
  if sample.shape[0]:
    data = np.insert(data, data.shape[0], sample, axis=0)

  if not ctr % 100:
    print(f"{ctr} samples captured")
  ctr += 1

port.close()

np.save("tof_data_empty_space.npy", data)

print("exiting")
