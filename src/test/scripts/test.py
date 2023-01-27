#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import PointStamped, Vector3
from random import random
from time import sleep

rospy.init_node("dummy_pub")
point = rospy.Publisher("/ultrasonic", PointStamped, queue_size=10)
vector = rospy.Publisher("/ultrasonic/vec", Vector3, queue_size=10)
pub_rate = rospy.Rate(10)

msg = PointStamped()
vec = Vector3()
while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "/robot"
    msg.point.x = 3 + random()
    msg.point.y = 0 + random()
    msg.point.z = 2 + random()

    vec.x, vec.y, vec.z = msg.point.x, msg.point.y, msg.point.z

    point.publish(msg)
    vector.publish(vec)

    pub_rate.sleep()
