#!/usr/bin/env python3
import rospy
import RPi.GPIO as gpio
from time import sleep, time
from std_msgs.msg import Float32

gpio.setmode(gpio.BOARD)

class Ultrasonic:
  def __init__(self, trigger, echo):
    self.trigger = trigger
    self.echo = echo

  def setup(self):
    gpio.setup(self.trigger, gpio.OUT)
    gpio.setup(self.echo, gpio.IN)

  def measure(self):
    # ensure the trigger is low to start
    gpio.output(self.trigger, gpio.LOW)
    sleep(2e-6)

    # make a 10 us pulse to send echo
    gpio.output(self.trigger, gpio.HIGH)
    sleep(10e-6)
    gpio.output(self.trigger, gpio.LOW)

    while gpio.input(self.echo) == 0:
      sleep(2e-6)
    start = time()

    while gpio.input(self.echo) == 1:
      dur = time() - start
      # do not exceed max range time
      if dur > 4 / 343:
        return 0
      sleep(2e-6)

    # assuming speed of sound is 343 m/s
    return dur * 343 / 2

rospy.init_node("ultrasonic_driver")

sensors = [
  Ultrasonic(7, 11),
]

rate = rospy.Rate(10)
pubs = [
  rospy.Publisher(f"/ultrasonic{i}", Float32, queue_size=10)
  for i in range(len(sensors))
]
msgs = [Float32() for i in range(len(sensors))]

try:
  for s in sensors:
    s.setup()

  while not rospy.is_shutdown():
    distances = [s.measure() for s in sensors]
    for d, p, m in zip(distances, pubs, msgs):
      m.data = d
      p.publish(m)
    rate.sleep()

finally:
  gpio.cleanup()
