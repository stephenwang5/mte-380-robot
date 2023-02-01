#!/usr/bin/env python3
import rospy
import utils
from mpu9250_jmdev.mpu_9250 import MPU9250
from geometry_msgs.msg import Vector3

rospy.init_node("accel_gyro_mpu_driver")
topics = {
  "/raw/lin_accel": Vector3,
  "/raw/gyro": Vector3,
  "/raw/mag": Vector3,
}
pubs = utils.create_pubs(topics)
msgs = utils.create_msgs(topics)

sensor = MPU9250()
sensor.configure()
sensor.calibrate()
sensor.configure()
rospy.loginfo("calibration done")

rate = rospy.Rate(20)
while not rospy.is_shutdown():
  utils.assign_vec(msgs["/raw/lin_accel"], *sensor.readAccelerometerMaster())
  utils.assign_vec(msgs["/raw/gyro"], *sensor.readGyroscopeMaster())
  utils.assign_vec(msgs["/raw/mag"], *sensor.readMagnetometerMaster())
  utils.publish_all(pubs, msgs)
  rate.sleep()
