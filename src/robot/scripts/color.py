#!/usr/bin/env python3
import rospy
import adafruit_tcs34725
import board
import utils
from math import pi
from std_msgs.msg import ColorRGBA
from robot.msg import ColorHSV

def rgb2hsv(r, g, b):
  r, g, b = r/255, g/255, b/255
  c_max = max(r, g, b)
  c_min = min(r, g, b)
  delta = c_max - c_min

  if delta == 0:
    h = 0
  elif c_max == r:
    h = 60 * ((g-b)/delta % 6)
  elif c_max == g:
    h = 60 * ((b-r)/delta + 2)
  elif c_max == b:
    h = 60 * ((r-g)/delta + 4)

  if c_max == 0:
    s = 0
  else:
    s = delta/c_max

  return h, s, c_max

rospy.init_node("color_driver")

i2c = board.I2C()
sensor = adafruit_tcs34725.TCS34725(i2c)
sensor.integration_time = 500
sensor.gain = 16

topics = {
  "/raw/color/rgb": ColorRGBA,
  "/raw/color/hsv": ColorHSV,
}
pubs = utils.create_pubs(topics)
msgs = utils.create_msgs(topics)

rate = rospy.Rate(1)
while not rospy.is_shutdown():
  rgb = msgs["/raw/color/rgb"]
  lux = sensor.lux
  rgb.a = lux if lux else 500
  raw_rgb = sensor.color_rgb_bytes
  rgb.r, rgb.g, rgb.b = list(map(lambda b : 500*b/rgb.a, raw_rgb))

  hsv = msgs["/raw/color/hsv"]
  hsv.hue, hsv.sat, hsv.val = rgb2hsv(rgb.r, rgb.g, rgb.b)

  utils.publish_all(pubs, msgs)
  rate.sleep()
