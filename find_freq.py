#!/usr/bin/env python3
import serial
from time import time
import signal

port = serial.Serial("/dev/cu.usbserial-AC00UOLH", baudrate=115200)

def handler(sig, frame):
  port.close()
signal.signal(signal.SIGINT, handler)

prevTime = time()
while True:
  port.readline()
  print(f"{1/(time() - prevTime):.2f} hz")
  prevTime = time()
