#!/usr/bin/env python3
import rospy
import utils
import numpy as np
from vl53l5cx.vl53l5cx import VL53L5CX
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray

def range2pc(x):
  spacing = 0.05
  y, z = np.mgrid[0:x.shape[0],0:x.shape[1]] * spacing
  middle = ((x.shape[0]-1) * spacing) / 2
  y = y - middle
  z = z - middle
  return np.stack([x,y,z], axis=2).reshape(-1,3)

def write_pc(points):
  dtype_size = np.dtype(np.float32).itemsize
  data = points.astype(np.float32).tobytes()

  fields = [PointField(
      name=n,
      offset=i * dtype_size,
      datatype=PointField.FLOAT32,
      count=1) for i, n in enumerate('xyz')]

  msg = msgs["/tof/pointcloud"]
  msg.header.frame_id = "tof_link"
  msg.header.stamp = rospy.Time.now()
  msg.height = 1
  msg.width = points.shape[0]
  msg.is_dense = True
  msg.is_bigendian = False
  msg.fields = fields
  msg.point_step = dtype_size * 3
  msg.row_step = dtype_size * 3 * points.shape[0]
  msg.data = data

topics = {
  "/tof/raw": Float32MultiArray,
  "/tof/pointcloud": PointCloud2,
}

rospy.init_node("tof_driver")
pubs = utils.create_pubs(topics)
msgs = utils.create_msgs(topics)

tof = VL53L5CX()

if not tof.is_alive():
  raise IOError("ToF not detected")

rospy.loginfo("initializing")
t = rospy.Time.now()
tof.init()
t = rospy.Time.now().to_sec() - t.to_sec()
rospy.loginfo(f"firmware upload took {t:.3f}")

tof.set_resolution(64)
tof.start_ranging()

rate = rospy.Rate(2)
data_ready_check = rospy.Rate(20)

while not rospy.is_shutdown():
  while not tof.check_data_ready():
    data_ready_check.sleep()
  
  rospy.loginfo("got new measurement")
  
  data = tof.get_ranging_data()
  distances = np.array(data.distance_mm[:64]).reshape(8,8) * 1e-3 # now in m
  distances = distances.T

  msgs["/tof/raw"].data = distances

  write_pc(range2pc(distances))

  utils.publish_all(pubs, msgs)
  
  rate.sleep()
