#include "imu_complementary_filter/complementary_filter_ros.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "ComplementaryFilterROS");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  imu_tools::ComplementaryFilterROS filter(nh, nh_private);
  ros::spin();
  return 0;
}
