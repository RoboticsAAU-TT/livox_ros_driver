#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
using LivoxCustomMsg = livox_ros_driver::CustomMsg;
using LivoxCustomMsgPtr = livox_ros_driver::CustomMsg::Ptr;
using LivoxCustomMsgConstPtr = livox_ros_driver::CustomMsg::ConstPtr;

namespace pointcloud2_to_livox {

class LivoxConverter {
public:
  LivoxConverter() {
    livox_msg.reset(new LivoxCustomMsg);
  }

  template <typename CustomMsg>
  LivoxCustomMsgConstPtr convert(const CustomMsg& points_msg) {

    livox_msg->header = points_msg.header;
    livox_msg->timebase = points_msg.header.stamp.nanoseconds(); //TODO: Verify if unit is correct
    livox_msg->point_num = points_msg.width;

    unsigned char* points_ptr = &points_msg.data.data();
    for (int i = 0; i < points_msg.width; i++) {
      livox_msg.points[i].x = *reinterpret_cast<float*>(points_ptr + points_msg.fields[0].offset);
      livox_msg.points[i].y = *reinterpret_cast<float*>(points_ptr + points_msg.fields[1].offset);
      livox_msg.points[i].z = *reinterpret_cast<float*>(points_ptr + points_msg.fields[2].offset);
      livox_msg.points[i].offset_time = *reinterpret_cast<std::uint32_t*>(points_ptr + points_msg.fields[3].offset);
      livox_msg.points[i].reflectivity = *reinterpret_cast<float*>(points_ptr + points_msg.fields[4].offset);
      livox_msg.points[i].tag = *reinterpret_cast<std::uint8_t*>(points_ptr + points_msg.fields[5].offset);
      livox_msg.points[i].line = *reinterpret_cast<std::uint8_t*>(points_ptr + points_msg.fields[6].offset);

      points_ptr += points_msg.point_step;
    }

    return livox_msg;
  }

private:
  LivoxCustomMsgPtr livox_msg;
};
}  // namespace pointcloud2_to_livox