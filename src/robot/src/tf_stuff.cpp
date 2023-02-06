#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

class TF {
  ros::NodeHandle* n;
  tf2_ros::TransformBroadcaster* br;
  ros::Subscriber sub;
  geometry_msgs::TransformStamped tf_msg;

  void callback(const geometry_msgs::Quaternion& msg) {
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = "imu_link";

    tf_msg.transform.translation.x = 1;
    tf_msg.transform.translation.y = 1;
    tf_msg.transform.translation.z = 0;

    tf_msg.transform.rotation = msg;

    br->sendTransform(tf_msg);
  }

public:

  TF(ros::NodeHandle* n, tf2_ros::TransformBroadcaster* br) :
    n(n), br(br), tf_msg() {
    sub = n->subscribe("/imu/filtered", 10, &TF::callback, this);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_broadcaster");
  ros::NodeHandle n;
  tf2_ros::TransformBroadcaster br;

  TF main_obj(&n, &br);

  ros::spin();
}