#!/usr/bin/env python3
import rospy
import tf
import utils
import numpy as np
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

position = np.array([0.]*3) # initialize robot position
velocity = np.array([0.]*3)
angle = np.array([0.]*3)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
  ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
  a = np.array([ax, ay, az])
  omega = np.array([gx, gy, gz])

  # numerical integration
  velocity += a * 0.1
  position += velocity * 0.1
  angle += omega * 0.1

  quat = tf.transformations.quaternion_from_euler(*angle)
  utils.assign_quat(msgs["/odom"].pose.pose.orientation, quat)
  utils.assign_vec(msgs["/odom"].pose.pose.position, *position)
  utils.assign_vec(msgs["/odom"].twist.twist.linear, *velocity)
  utils.assign_vec(msgs["/odom"].twist.twist.angular, *omega)
  msgs["/odom"].header.stamp = rospy.Time.now()

  utils.assign_vec(msgs["/raw/accel"].linear, ax, ay, az)

  utils.assign_vec(msgs["/raw/gyro"].angular, gx, gy, gz)

  utils.publish_all(pubs, msgs)

  rate.sleep()
