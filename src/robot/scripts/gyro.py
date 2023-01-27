#!/usr/bin/env python3
import rospy
import tf
import utils
from icm20948 import ICM20948
from geometry_msgs.msg import Accel, Vector3, Twist
from nav_msgs.msg import Odometry

rospy.init_node("accel_gyro_driver")
imu = ICM20948(0x69)

topics = {
  "/odom": Odometry,
  "/raw/accel": Accel,
  "/raw/gyro": Twist,
}

pubs = utils.create_pubs(topics)
msgs = utils.create_msgs(topics)

msgs["/odom"].header.frame_id = "world"
msgs["/odom"].child_frame_id = "robot"

rate = rospy.Rate(10)
while not rospy.is_shutdown():
  ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()

  quat = tf.transformations.quaternion_from_euler(gx, gy, gz)
  utils.assign_quat(msgs["/odom"].pose.pose.orientation, quat)

  utils.assign_vec(msgs["/raw/accel"].linear, ax, ay, az)

  # TODO: numerically integrate linear accel to get linear twist (vel)
  utils.assign_vec(msgs["/raw/gyro"].angular, gx, gy, gz)

  utils.publish_all(pubs, msgs)

  rate.sleep()
