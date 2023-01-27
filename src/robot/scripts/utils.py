import rospy

def create_pubs(topics: dict):
  pubs = {}
  for k, V in topics.items():
    pubs[k] = rospy.Publisher(k, V, queue_size=10)
  return pubs

def create_msgs(topics: dict):
  return {k: V() for k, V in topics.items()}

def publish_all(pubs: dict, msgs: dict):
  for topic, p in pubs.items():
    p.publish(msgs[topic])

def assign_quat(msg, quat):
  msg.x = quat[0]
  msg.y = quat[1]
  msg.z = quat[2]
  msg.w = quat[3]

def assign_vec(msg, *vec):
  msg.x = vec[0]
  msg.y = vec[1]
  msg.z = vec[2]
