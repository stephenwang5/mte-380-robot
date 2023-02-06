#!/usr/bin/env python3
import rospy
import utils
import tf
from mpu9250_jmdev.mpu_9250 import MPU9250
from robot.msg import RawImu

rospy.init_node("accel_gyro_mpu_driver")
topics = {
  "/imu/raw": RawImu,
}
pubs = utils.create_pubs(topics)
msgs = utils.create_msgs(topics)

sensor = MPU9250()
sensor.configure()
sensor.calibrate(retry=1)
sensor.configure()
# sensor.mbias = rospy.get_param("mag_bias")
# sensor.abias = rospy.get_param("accel_bias")
# sensor.gbias = rospy.get_param("gyro_bias")
# if not sensor.mbias or not sensor.abias or not sensor.gbias:
#   rospy.logerr("imu params not complete")
#   raise Exception("imu params not complete")
rospy.loginfo("calibration done")

rate = rospy.Rate(50)
while not rospy.is_shutdown():
  imu_msg = msgs["/imu/raw"]
  utils.assign_vec(imu_msg.lin_accel, *sensor.readAccelerometerMaster())
  utils.assign_vec(imu_msg.ang_vel, *sensor.readGyroscopeMaster())

  mag = sensor.readMagnetometerMaster()
  utils.assign_vec(msgs["/imu/raw"].mag_density, *mag)

  utils.publish_all(pubs, msgs)
  rate.sleep()
