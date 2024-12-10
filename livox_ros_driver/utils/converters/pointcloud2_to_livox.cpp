#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

#include <utils/converters/livox_converter.hpp>

namespace livox_to_pointcloud2 {

class PointCloud2ToLivox {
public:
  PointCloud2ToLivox() : nh("~") {
    points_pub = nh.advertise<livox_ros_driver::CustomMsg>("/livox/lidar", 10);
    points_sub = nh.subscribe("/livox/points", 10, &PointCloud2ToLivox::callback, this);
  }

  void callback(const sensor_msgs::PointCloud2::ConstPtr& points_msg) {
    const auto livox_msg = converter.convert(*points_msg);
    points_pub.publish(livox_msg);
  }

private:
  ros::NodeHandle nh;
  ros::Publisher points_pub;
  ros::Subscriber points_sub;

  LivoxConverter converter;
};

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointcloud2_to_livox");
  livox_to_pointcloud2::PointCloud2ToLivox node;
  ros::spin();

  return 0;
}