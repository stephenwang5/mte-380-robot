#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped

rospy.init_node("tf_broadcaster")
broadcaster = tf2_ros.TransformBroadcaster()

trf = TransformStamped()
trf.header.frame_id = "/world"
trf.child_frame_id = "/robot"

trf.transform.translation.x = 0
trf.transform.translation.y = 0
trf.transform.translation.z = 0

rot = tf.transformations.quaternion_from_euler(0, 0, 0)
trf.transform.rotation.x = rot[0]
trf.transform.rotation.y = rot[1]
trf.transform.rotation.z = rot[2]
trf.transform.rotation.w = rot[3]

rate = rospy.Rate(1)

while not rospy.is_shutdown():
  trf.header.stamp = rospy.Time.now()
  broadcaster.sendTransform(trf)
  rate.sleep()
